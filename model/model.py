import torch
import torch.nn as nn
from bone import bone_parents, bone_children

def position_encoding(x, num_freqs):
    """
    Args:
      x: Tensor[..., D]  input
      num_freqs: int   num of frequency band  L
    Returns:
      Tensor[..., D * 2 * num_freqs]
    """
    x = x.unsqueeze(-1)  # [..., D, 1]
    freqs = 2.0 ** torch.arange(num_freqs, device=x.device) * torch.pi  # [L]
    # [..., D, L]
    x_freq = x * freqs  
    # sin, cos  [..., D, L]
    sin = torch.sin(x_freq)
    cos = torch.cos(x_freq)
    # concat → [..., D, 2L] → flatten → [..., D*2L]
    pe = torch.cat([sin, cos], dim=-1).flatten(-2, -1)
    return pe  # shape [..., D*2L]


class Skeleton2Mesh(nn.Module):
    def __init__(self,
                 num_joints=20,
                 num_bones=19,
                 gsd_dim=100,
                 mlp_hidden=[256, 256],
                 pe_freqs_bone=5,
                 pe_freqs_order=2):
        super().__init__()
        self.num_bones = num_bones


        self.orientation_embed = nn.Linear(3,3)

        # self.orientation_embed = nn.Sequential(
        #     nn.Linear(3,12),
        #     nn.GELU(),
        #     nn.Linear(12,12),
        #     nn.GELU(),
        #     nn.Linear(12,3)
        # )

        # 1) Global Spatial Descriptor
        self.gsd_mlp = nn.Sequential(
            nn.Linear(num_joints*3, mlp_hidden[0]),
            nn.GELU(),
            nn.Linear(mlp_hidden[0], mlp_hidden[1]),
            nn.GELU(),
            nn.Linear(mlp_hidden[1], gsd_dim)
        )

        # 2) Positional Encoding params
        self.pe_freqs_bone = pe_freqs_bone
        self.pe_freqs_order = pe_freqs_order

        # 3) Define 4 separate MLP heads
        in_dim = pe_freqs_bone * 2 * 6 + pe_freqs_order * 2 * num_bones + gsd_dim + 3

        def make_head(out_dim: int) -> nn.Sequential:
            return nn.Sequential(
                nn.Linear(in_dim, mlp_hidden[0]),
                nn.LeakyReLU(),
                nn.Linear(mlp_hidden[0], mlp_hidden[1]),
                nn.LeakyReLU(),
                nn.Linear(mlp_hidden[1], out_dim)
            )

        # two heads output dim=3, two heads output dim=1
        self.head1 = make_head(1)
        self.head2 = make_head(1)
        self.head3 = make_head(1)

        self.pool1 = nn.Linear(num_bones, 1, bias=True)
        self.pool2 = nn.Linear(num_bones, 1, bias=True)
        self.pool3 = nn.Linear(num_bones, 1, bias=True)

    def forward(self, skeletons_data: torch.Tensor) -> torch.Tensor:
        B = skeletons_data.shape[0]
        orientation = skeletons_data[:,-3:]
        skeletons_flat = skeletons_data[:,:-3]
        # reshape flat input → [B, num_bones, 3]
        skeletons = skeletons_flat.view(B, -1, 3)  

        # compute bone endpoints → [B, num_bones, 6]
        parents  = skeletons[:, bone_parents, :]
        children = skeletons[:, bone_children, :]
        Bk = torch.cat([parents, children], dim=-1)

        # positional encoding
        pe_bk = position_encoding(Bk, self.pe_freqs_bone)   # [B, num_bones, 60]
        ok    = torch.eye(self.num_bones, device=skeletons.device)\
                    .unsqueeze(0).expand(B, -1, -1)
        pe_ok = position_encoding(ok, self.pe_freqs_order)  # [B, num_bones, 80]

        orientation_embed = self.orientation_embed(orientation).unsqueeze(1)
        orientation_embed = orientation_embed.expand(-1,self.num_bones,-1)

        # global spatial descriptor g → [B, num_bones, gsd_dim]
        flat = skeletons.view(B, -1)
        g    = self.gsd_mlp(flat).unsqueeze(1)\
                  .expand(-1, self.num_bones, -1)

        # bone-level feature OE → [B, num_bones, in_dim]
        OE = torch.cat([pe_bk, pe_ok, g, orientation_embed], dim=-1)

        # 4 heads
        out1 = self.head1(OE)  # [B, num_bones, 3] # index AA
        out2 = self.head2(OE)  # [B, num_bones, 3] # inter AA
        out3 = self.head3(OE)  # [B, num_bones, 1] # ring AA

        

        # 1) permute to put 'num_bones' in last dim for heads with C>1
        #    and apply linear pooling:
        o1 = out1.squeeze(-1)            # [B, num_bones]
        agg1 = self.pool1(o1)     # [B, 1]

        o2 = out2.squeeze(-1)            # [B, num_bones]
        agg2 = self.pool2(o2)     # [B, 1]

        
        o3 = out3.squeeze(-1)                 # [B, num_bones]
        agg3 = self.pool3(o3)                 # [B] → will become [B,1] after unsqueeze

        # 2) concat to 4dim vector
        angles = torch.cat([agg1, agg2, agg3], dim=-1)  # [B,3]
        

        return angles