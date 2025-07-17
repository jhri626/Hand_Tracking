import torch
import torch.nn as nn

class DHAmodel(nn.Module):
    def __init__(self,
                 mlp_hidden=[18432, 8192, 256, 22],
                 ):
        super().__init__()
        
        self.orientation_embed = nn.Linear(3,3)

        # self.orientation_embed = nn.Sequential(
        #     nn.Linear(3,12),
        #     nn.GELU(),
        #     nn.Linear(12,12),
        #     nn.GELU(),
        #     nn.Linear(12,3)
        # )

        # 1) Global Spatial Descriptor

        # 2) Positional Encoding params

        # 3) Define 4 separate MLP heads
        in_dim = 63

        def make_head(out_dim: int) -> nn.Sequential:
            return nn.Sequential(
                nn.Linear(in_dim, mlp_hidden[0]),
                nn.LeakyReLU(),
                nn.Linear(mlp_hidden[0], mlp_hidden[1]),
                nn.ReLU(),
                nn.Linear(mlp_hidden[1], mlp_hidden[2]),
                nn.ReLU(),
                nn.Linear(mlp_hidden[2], mlp_hidden[3]),
                nn.ReLU(),
                nn.Linear(mlp_hidden[3], out_dim)
            )

        # two heads output dim=3, two heads output dim=1
        self.joint_module = make_head(3)
        
    def forward(self, skeletons_data: torch.Tensor) -> torch.Tensor:
        B = skeletons_data.shape[0]
        orientation = skeletons_data[:,-3:]        
        skeletons = skeletons_data[:,:-3]
        # reshape flat input → [B, num_bones, 3]

        # compute bone endpoints → [B, num_bones, 6]
        orientation_embed = self.orientation_embed(orientation)
        print
        # bone-level feature OE → [B, in_dim]
        OE = torch.cat([skeletons, orientation_embed], dim=-1)

        # 4 heads
        out = self.joint_module(OE)  # [B, num_bones, 3] # index AA
               # [B] → will become [B,1] after unsqueeze        

        return out