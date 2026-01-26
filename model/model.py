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
    # concat -> [..., D, 2L] -> flatten -> [..., D*2L]
    pe = torch.cat([sin, cos], dim=-1).flatten(-2, -1)
    return pe  # shape [..., D*2L]


class FiLMInteraction(nn.Module):
    """
    Feature-wise Linear Modulation (FiLM) Layer.
    """
    def __init__(self, feature_dim, cond_dim):
        super().__init__()
        
        self.cond_mlp = nn.Sequential(
            nn.Linear(cond_dim, feature_dim * 2),
            nn.LeakyReLU(),
            nn.Linear(feature_dim * 2, feature_dim * 2)
        )
        
        # Zero initialization ensures identity mapping at the start of training.
        nn.init.zeros_(self.cond_mlp[-1].weight)
        nn.init.zeros_(self.cond_mlp[-1].bias)

    def forward(self, features, condition):
        modulation = self.cond_mlp(condition) # [B, 2*D]
        gamma, beta = torch.chunk(modulation, 2, dim=-1)
        out = features * (1.0 + gamma) + beta
        return out


class BoneInteraction(nn.Module):
    """
    Lightweight module to exchange information between bones.
    """
    def __init__(self, num_bones):
        super().__init__()
        self.mixing = nn.Sequential(
            nn.Linear(num_bones, num_bones),
            nn.LeakyReLU(), 
            nn.Linear(num_bones, num_bones)
        )
        nn.init.zeros_(self.mixing[-1].weight)

    def forward(self, x):
        x_T = x.transpose(1, 2)
        delta = self.mixing(x_T)
        out = x + delta.transpose(1, 2)
        return out


class Skeleton2Mesh(nn.Module):
    def __init__(self,
                 num_joints=20,
                 num_bones=19,
                 gsd_dim=100,
                 mlp_hidden=[256, 256],
                 pe_freqs_bone=5,
                 pe_freqs_order=2,
                 use_euler = True,
                 use_quat = False,
                 use_6d = False):
        super().__init__()
        self.num_bones = num_bones
        self.use_euler = use_euler
        self.use_quat = use_quat
        self.use_6d = use_6d

        # 1. Determine if orientation is used and set embedding dimension
        self.use_orientation = self.use_euler or self.use_quat or self.use_6d
        
        if self.use_orientation:
            if self.use_euler:
                self.emb_dim = 3
            elif self.use_quat:
                self.emb_dim = 4
            elif self.use_6d:
                self.emb_dim = 6
            
            # Initialize FiLM only if orientation is used
            self.film_interaction = FiLMInteraction(feature_dim=gsd_dim, cond_dim=self.emb_dim)
        else:
            # No orientation used
            self.emb_dim = 0
            self.film_interaction = None

        # 2. Global Spatial Descriptor (Landmark Features)
        self.gsd_mlp = nn.Sequential(
            nn.Linear(num_joints*3, mlp_hidden[0]),
            nn.GELU(),
            nn.Linear(mlp_hidden[0], mlp_hidden[1]),
            nn.GELU(),
            nn.Linear(mlp_hidden[1], gsd_dim)
        )

        # 3. Positional Encoding params
        self.pe_freqs_bone = pe_freqs_bone
        self.pe_freqs_order = pe_freqs_order

        # 4. Calculate Input Dimension for Shared Backbone
        pe_bk_dim = pe_freqs_bone * 2 * 6
        pe_ok_dim = pe_freqs_order * 2 * num_bones
        
        # in_dim is consistent regardless of fusion (since fusion happens inside gsd_dim)
        in_dim = pe_bk_dim + pe_ok_dim + gsd_dim

        # 5. Interaction Module
        self.interaction = BoneInteraction(num_bones)

        # 6. Shared Backbone
        self.shared_backbone = nn.Sequential(
            nn.Linear(in_dim, mlp_hidden[0]),
            nn.LeakyReLU(),
            nn.Linear(mlp_hidden[0], mlp_hidden[1]),
            nn.LeakyReLU()
        )

        # 7. Lightweight Heads
        head_in_dim = mlp_hidden[1]
        
        self.head1 = nn.Linear(head_in_dim, 1)
        self.head2 = nn.Linear(head_in_dim, 1)
        self.head3 = nn.Linear(head_in_dim, 1)

        self.pool1 = nn.Linear(num_bones, 1, bias=True)
        self.pool2 = nn.Linear(num_bones, 1, bias=True)
        self.pool3 = nn.Linear(num_bones, 1, bias=True)


    def forward(self, skeletons_data: torch.Tensor) -> torch.Tensor:
        B = skeletons_data.shape[0]
        
        # --- [MODIFIED] Branching based on orientation usage ---
        if self.use_orientation:
            # Split orientation and skeleton data
            orientation = skeletons_data[:, -self.emb_dim:] # [B, emb_dim]
            skeletons_flat = skeletons_data[:, :-self.emb_dim]
        else:
            # Use data as is (assuming input is purely skeletons)
            orientation = None
            skeletons_flat = skeletons_data[:, :-3]

        # Reshape flat input -> [B, num_bones, 3]
        skeletons = skeletons_flat.view(B, -1, 3)  

        # Compute bone endpoints -> [B, num_bones, 6]
        parents  = skeletons[:, bone_parents, :]
        children = skeletons[:, bone_children, :]
        Bk = torch.cat([parents, children], dim=-1)

        # --- Positional Encoding ---
        pe_bk = position_encoding(Bk, self.pe_freqs_bone)
        
        ok = torch.eye(self.num_bones, device=skeletons.device)\
                    .unsqueeze(0).expand(B, -1, -1)
        pe_ok = position_encoding(ok, self.pe_freqs_order)

        # --- Global Spatial Descriptor & Optional FiLM ---
        flat = skeletons.view(B, -1)
        
        # 1. Extract GSD features
        g_feature = self.gsd_mlp(flat) # [B, gsd_dim]
        
        # 2. Apply FiLM or Pass Through
        if self.use_orientation and self.film_interaction is not None:
            # Case: Orientation used -> Apply Modulation
            g_final = self.film_interaction(g_feature, orientation)
        else:
            # Case: No Orientation -> Pass raw GSD
            g_final = g_feature
        
        # 3. Expand to bone dimension
        g_expanded = g_final.unsqueeze(1).expand(-1, self.num_bones, -1)

        # Construct bone-level feature OE
        OE = torch.cat([pe_bk, pe_ok, g_expanded], dim=-1)

        # --- Bone Interaction & Backbone ---
        OE = self.interaction(OE)
        features = self.shared_backbone(OE)

        # --- Heads & Pooling ---
        out1 = self.head1(features)
        out2 = self.head2(features)
        out3 = self.head3(features)

        agg1 = self.pool1(out1.squeeze(-1)) 
        agg2 = self.pool2(out2.squeeze(-1))
        agg3 = self.pool3(out3.squeeze(-1))

        angles = torch.cat([agg1, agg2, agg3], dim=-1) 
        
        return angles