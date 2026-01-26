#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 node for model hand-angle inference (NumPy only).
Updated to match the new PyTorch architecture with FiLM and Shared Backbone.
Compatible with Python 3 (ROS 2).
"""

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from scipy.special import erf
from bone import bone_parents, bone_children   # length 19
from vr.msg import HandSyncData
from scipy.signal import butter, lfilter, lfilter_zi

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
MODEL_WEIGHTS_PATH = r'C:/Users/dyros/Desktop/dummy_ws/model/best_val_loss_rot6d.npz'

NUM_JOINTS = 20
NUM_BONES  = 19
PE_FREQ_BK = 5
PE_FREQ_OK = 2
GSD_DIM    = 100
NEG_SLOPE  = 0.01                 # LeakyReLU slope
ORI_DIM    = 6                    # rot6d (Updated from 3)

INPUT_TOPIC  = '/hand_sync_data'
OUTPUT_TOPIC = '/model_out'
KEYWORDS     = [0, 21, 22, 23, 24, 25]   # joints to skip
RADTODEG = 180.0 / np.pi

# --------------------------------------------------------------------------- #
# NumPy model (Skeleton2Mesh replica)
# --------------------------------------------------------------------------- #
class Skeleton2AngleNumPy(object):
    """Pure-NumPy implementation reproducing the NEW PyTorch Skeleton2Mesh with FiLM."""
    def __init__(self, npz_path):
        # Load weights
        try:
            loaded = np.load(npz_path, allow_pickle=True)
        except:
            loaded = np.load(npz_path)
            
        self.params = {}
        for k, v in loaded.items():
            self.params[k] = v.astype(np.float32)
            
        print(f"Loaded {len(self.params)} parameters.")

    # --------------------------------------------------------------------- #
    # Basic ops
    # --------------------------------------------------------------------- #
    @staticmethod
    def gelu(x):
        # PyTorch GELU approximation
        return 0.5 * x * (1.0 + erf(x / np.sqrt(2.0)))

    @staticmethod
    def leaky_relu(x, neg_slope=NEG_SLOPE):
        return np.where(x >= 0, x, neg_slope * x)

    def linear(self, x, prefix):
        """
        Applies Linear layer: xW^T + b
        """
        w = self.params[prefix + '.weight']
        b = self.params[prefix + '.bias']
        return np.dot(x, w.T) + b

    # --------------------------------------------------------------------- #
    # Positional encoding
    # --------------------------------------------------------------------- #
    @staticmethod
    def position_encoding(x, num_freqs):
        """
        x: ndarray [..., D]
        returns ndarray [..., D * 2 * num_freqs]
        """
        # freqs: (L,)
        freqs = (2.0 ** np.arange(num_freqs, dtype=x.dtype)) * np.pi 
        x_exp = x[..., None] * freqs                                       # [..., D, L]
        sin   = np.sin(x_exp)
        cos   = np.cos(x_exp)
        pe    = np.concatenate([sin, cos], axis=-1)                        # [..., D, 2L]
        
        # Flatten last 2 dims: (..., D * 2L)
        new_shape = x.shape[:-1] + (-1,)
        return pe.reshape(new_shape)

    # --------------------------------------------------------------------- #
    # Module: FiLM Interaction (New)
    # --------------------------------------------------------------------- #
    def film_interaction(self, features, condition):
        """
        Mimics FiLMInteraction module.
        features:  [B, gsd_dim]
        condition: [B, cond_dim] (orientation)
        """
        # cond_mlp: Linear -> LeakyReLU -> Linear
        x = self.linear(condition, 'film_interaction.cond_mlp.0')
        x = self.leaky_relu(x)
        modulation = self.linear(x, 'film_interaction.cond_mlp.2') # [B, gsd_dim * 2]
        
        # Split into gamma, beta
        # modulation shape is (B, 200) if gsd_dim=100
        gamma, beta = np.split(modulation, 2, axis=-1)
        
        # Apply FiLM: out = features * (1 + gamma) + beta
        out = features * (1.0 + gamma) + beta
        return out

    # --------------------------------------------------------------------- #
    # Module: BoneInteraction
    # --------------------------------------------------------------------- #
    def bone_interaction(self, x):
        """
        Mimics BoneInteraction module.
        Input x: [B, num_bones, dim]
        Logic: Transpose -> Mixing MLP -> Transpose -> Residual
        """
        # 1. Transpose: [B, num_bones, dim] -> [B, dim, num_bones]
        x_T = x.transpose(0, 2, 1)

        # 2. Mixing (MLP on num_bones dimension)
        # interaction.mixing: Linear -> LeakyReLU -> Linear
        h = self.linear(x_T, 'interaction.mixing.0')
        h = self.leaky_relu(h)
        delta = self.linear(h, 'interaction.mixing.2')

        # 3. Transpose back + Residual
        return x + delta.transpose(0, 2, 1)

    # --------------------------------------------------------------------- #
    # Forward pass
    # --------------------------------------------------------------------- #
    def forward(self, skeletons_data):
        """
        Input : skeletons_data (B, 60 + ORI_DIM)
        Output: (B, 3)
        """
        B = skeletons_data.shape[0]

        # 1. Split input (Orientation is at the end)
        ori = skeletons_data[:, -ORI_DIM:]             # (B, ORI_DIM)
        flat_joints = skeletons_data[:, :-ORI_DIM]     # (B, 60)

        # --- Data Prep ---
        skel = flat_joints.reshape(B, NUM_JOINTS, 3)                # (B, 20, 3)

        # Bone endpoints
        parents  = skel[:, bone_parents, :]                         # (B, 19, 3)
        children = skel[:, bone_children, :]
        Bk = np.concatenate([parents, children], axis=-1)           # (B, 19, 6)

        # Positional Encodings
        pe_bk = self.position_encoding(Bk, PE_FREQ_BK)              # (B, 19, 60)

        eye_nb = np.eye(NUM_BONES, dtype=np.float32)
        ok     = np.broadcast_to(eye_nb, (B, NUM_BONES, NUM_BONES)) # (B, 19, 19)
        pe_ok  = self.position_encoding(ok, PE_FREQ_OK)             # (B, 19, 76)

        # --- Global Spatial Descriptor (GSD) & FiLM ---
        flat = skel.reshape(B, -1)                                  # (B, 60)
        
        # gsd_mlp: Linear->GELU->Linear->GELU->Linear
        g = self.linear(flat, 'gsd_mlp.0')
        g = self.gelu(g)
        g = self.linear(g, 'gsd_mlp.2')
        g = self.gelu(g)
        g_feature = self.linear(g, 'gsd_mlp.4')                     # (B, 100)

        # Apply FiLM (Modulate GSD with Orientation)
        g_final = self.film_interaction(g_feature, ori)             # (B, 100)

        # Expand to bone dimension
        # (B, 100) -> (B, 1, 100) -> (B, 19, 100)
        g_expanded = g_final[:, None, :].repeat(NUM_BONES, axis=1)

        # Construct OE feature
        # New: [pe_bk, pe_ok, g_expanded]
        OE = np.concatenate([pe_bk, pe_ok, g_expanded], axis=-1)

        # --- Bone Interaction ---
        OE = self.bone_interaction(OE)

        # --- Shared Backbone ---
        # shared_backbone: Linear -> LeakyReLU -> Linear -> LeakyReLU
        features = self.linear(OE, 'shared_backbone.0')
        features = self.leaky_relu(features)
        features = self.linear(features, 'shared_backbone.2')
        features = self.leaky_relu(features)                        # (B, 19, hidden_dim)

        # --- Lightweight Heads ---
        out1 = self.linear(features, 'head1') # (B, 19, 1)
        out2 = self.linear(features, 'head2') # (B, 19, 1)
        out3 = self.linear(features, 'head3') # (B, 19, 1)

        # --- Pooling along bone dim ---
        o1 = out1.squeeze(-1) # (B, 19)
        o2 = out2.squeeze(-1)
        o3 = out3.squeeze(-1)

        # pool layers: Linear(19, 1)
        agg1 = self.linear(o1, 'pool1')
        agg2 = self.linear(o2, 'pool2')
        agg3 = self.linear(o3, 'pool3')

        # Concat -> (B, 3)
        return np.concatenate([agg1, agg2, agg3], axis=-1).astype(np.float32)


# --------------------------------------------------------------------------- #
# ROS Node Wrapper
# --------------------------------------------------------------------------- #
class InferenceNode(Node):
    """ROS 2 node that embeds Skeleton2AngleNumPy and publishes 8-D predictions."""
    def __init__(self):
        super().__init__('skeleton2angle_inference')

        self.declare_parameter('mode', 'model')
        self.mode = self.get_parameter('mode').value

        # Load model
        try:
            self.model = Skeleton2AngleNumPy(MODEL_WEIGHTS_PATH)
            self.get_logger().info('✅ Loaded weights from %s' % MODEL_WEIGHTS_PATH)
        except Exception as e:
            self.get_logger().error('Model load failed: %s' % e)
            self.model = None
            return
        
        # --- Butterworth filter design ----------------------------------
        order      = 2        # 2nd-order IIR
        fs         = 60.0     # Callback frequency (Hz)
        fc         = 10.0     # cutoff frequency (Hz)
        nyq        = 0.5 * fs
        normal_cut = fc / nyq

        self.b, self.a = butter(order, normal_cut, btype='low', analog=False)
        zi_base        = lfilter_zi(self.b, self.a)
        
        # initialize filter state for 3d vector
        self.zi_filter = [zi_base * 0.0 for _ in range(3)]
        
        # Exponential Moving Average parameters
        self.ema_alpha = 0.1
        self.ema       = None  # stores previous EMA value, shape (3,)

        # ROS I/O
        self.pub = self.create_publisher(Float32MultiArray, OUTPUT_TOPIC, 1)
        self.pub_data = self.create_publisher(Float32MultiArray, '/model_out_data', 1)
        self.subscription = self.create_subscription(
            HandSyncData, 
            INPUT_TOPIC,
            self.callback,
            1)
        self.get_logger().info('Node ready - waiting for %s' % INPUT_TOPIC)

    # --------------------------------------------------------------------- #
    def _flatten_posearray(self, msg):
        """PoseArray -> flat list of 60 floats (skip KEYWORDS joints)."""
        data = []
        for idx, p in enumerate(msg.poses):
            if idx in KEYWORDS:
                continue
            data.extend([
                p.position.x, p.position.y, p.position.z
            ])
        return np.asarray(data, np.float32)

    # --------------------------------------------------------------------- #
    def callback(self, msg):
        if self.model is None:
            return
        
        
        try:
            # 1. Flatten Joints
            x_flat = self._flatten_posearray(msg.pose_array)[None, :]             
            
            # 2. Extract Orientation (Last ORI_DIM elements)
            extra = np.array(msg.angles[-ORI_DIM:], dtype=np.float32).reshape(1, ORI_DIM)
            
            # 3. Concatenate
            x_input = np.concatenate([x_flat, extra], axis=1) # (1, 60 + ORI_DIM)

            if x_input.shape[1] != NUM_JOINTS * 3 + ORI_DIM:
                self.get_logger().warn('Unexpected input length %d' % x_input.shape[1])
                return

            # 4. Inference
            if self.mode == "model":
                out = self.model.forward(x_input)                           
                out_vec = out.flatten()                                     

                # 5. Filtering (LPF)
                filtered = np.zeros_like(out_vec)
                for i in range(3):
                    y, self.zi_filter[i] = lfilter(
                        self.b, self.a,
                        [out_vec[i]],
                        zi=self.zi_filter[i]
                    )
                    filtered[i] = y[0]

                # 6. EMA
                if self.ema is None:
                    self.ema = filtered.copy()
                else:
                    self.ema = (
                        self.ema_alpha * filtered
                        + (1.0 - self.ema_alpha) * self.ema
                    )

                # 7. Construct Output (Apply Rad->Deg conversion)
                final_out = np.array(msg.angles[:-ORI_DIM], dtype=np.float32).reshape(8)
                final_out[1:4] = self.ema * RADTODEG
            elif self.mode == "baseline":
                final_out = np.array(msg.angles[:-ORI_DIM], dtype=np.float32).reshape(8)
                final_out[1:4] = final_out[1:4] * RADTODEG
            else:
                self.get_logger().warn(f'Unknown mode: {self.mode}')
                return



            # 8. Publish
            # Combine prediction with raw orientation for debugging
            data_out = np.concatenate([final_out, extra.squeeze()], axis=0)
            
            self.pub.publish(Float32MultiArray(data=final_out.tolist()))
            self.pub_data.publish(Float32MultiArray(data=data_out.tolist()))

        except Exception as e:
            self.get_logger().error('Inference error: %s' % e)

# --------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()