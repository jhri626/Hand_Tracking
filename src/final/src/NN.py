#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS 1 inference node for Skeleton2Mesh-based hand angle regression.

Key points
- Pure NumPy implementation (Python 2.7 compatible)
- Mirrors the NEW PyTorch architecture:
    * Global Spatial Descriptor (GSD)
    * FiLM conditioning using orientation (rot6d)
    * Bone interaction module
    * Shared backbone + lightweight heads
- Real-time ROS pipeline:
    * Subscribe HandSyncData
    * Run inference
    * Apply temporal smoothing (Butterworth LPF + EMA)
    * Publish final output (degrees)

Inputs
- msg.pose_array: PoseArray-like container of joint poses
- msg.angles: includes orientation (last ORI_DIM elements)

Outputs
- /model_out: 8-dim Float32MultiArray (with indices 1:4 overwritten by model output)
- /model_out_data: debug array (final_out + orientation)
"""

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray
from scipy.special import erf
from scipy.signal import butter, lfilter, lfilter_zi

from bone import bone_parents, bone_children  # length 19
from vr.msg import HandSyncData


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #

# Exported from the NEW PyTorch model (state_dict -> npz)
MODEL_WEIGHTS_PATH = r'C:/Users/dyros/Desktop/dummy_ws/model/best_val_loss_rot6d.npz'

# Skeleton topology
NUM_JOINTS = 20
NUM_BONES = 19

# Positional encoding frequency bands
PE_FREQ_BK = 5  # bone endpoints (parents/children)
PE_FREQ_OK = 2  # one-hot bone identity

# Model dimensions
GSD_DIM = 100
ORI_DIM = 6  # rot6d
NEG_SLOPE = 0.01  # LeakyReLU negative slope

# ROS topics
INPUT_TOPIC = '/hand_sync_data'
OUTPUT_TOPIC = '/model_out'

# Joints to skip when flattening PoseArray
KEYWORDS = [0, 21, 22, 23, 24, 25]

# Unit conversion
radTodeg = 180.0 / np.pi


# --------------------------------------------------------------------------- #
# NumPy model (Skeleton2Mesh replica)
# --------------------------------------------------------------------------- #

class Skeleton2AngleNumPy(object):
    """
    NumPy-only forward implementation of the NEW PyTorch Skeleton2Mesh model.

    Modules mirrored from PyTorch:
    - gsd_mlp: Linear -> GELU -> Linear -> GELU -> Linear
    - film_interaction: cond_mlp produces (gamma, beta), applies FiLM to GSD
    - bone_interaction: mixes along bone dimension with residual
    - shared_backbone: Linear -> LeakyReLU -> Linear -> LeakyReLU
    - head1/head2/head3 + pool1/pool2/pool3: per-axis heads with bone pooling
    """

    def __init__(self, npz_path):
        """
        Load weights from .npz (exported from PyTorch state_dict).
        All parameters are converted to float32 for consistent NumPy inference.
        """
        try:
            loaded = np.load(npz_path, allow_pickle=True)
        except Exception:
            loaded = np.load(npz_path)

        self.params = {}
        for k, v in loaded.items():
            self.params[k] = v.astype(np.float32)

        rospy.loginfo("Loaded %d parameters.", len(self.params))

    # ------------------------------ Basic ops ------------------------------ #

    @staticmethod
    def gelu(x):
        """GELU activation (erf-based), consistent with PyTorch default."""
        return 0.5 * x * (1.0 + erf(x / np.sqrt(2.0)))

    @staticmethod
    def leaky_relu(x, neg_slope=NEG_SLOPE):
        """LeakyReLU activation."""
        return np.where(x >= 0, x, neg_slope * x)

    def linear(self, x, prefix):
        """
        Linear layer: y = x @ W^T + b

        Args:
            x: input array (..., in_features)
            prefix: parameter prefix used in the exported PyTorch state_dict

        Returns:
            y: output array (..., out_features)
        """
        w = self.params[prefix + '.weight']
        b = self.params[prefix + '.bias']
        return np.dot(x, w.T) + b

    # -------------------------- Positional encoding ------------------------ #

    @staticmethod
    def position_encoding(x, num_freqs):
        """
        Sinusoidal positional encoding.

        Args:
            x: (..., D)
            num_freqs: number of frequency bands

        Returns:
            (..., D * 2 * num_freqs)
        """
        freqs = (2.0 ** np.arange(num_freqs, dtype=x.dtype)) * np.pi  # (L,)
        x_exp = x[..., None] * freqs                                   # (..., D, L)
        sin = np.sin(x_exp)
        cos = np.cos(x_exp)
        pe = np.concatenate([sin, cos], axis=-1)                      # (..., D, 2L)

        new_shape = x.shape[:-1] + (-1,)
        return pe.reshape(new_shape)

    # --------------------------- FiLM interaction -------------------------- #

    def film_interaction(self, features, condition):
        """
        FiLM (Feature-wise Linear Modulation) on GSD.

        features : (B, GSD_DIM)
        condition: (B, ORI_DIM)

        cond_mlp:
            Linear -> LeakyReLU -> Linear  => (B, 2 * GSD_DIM)

        Applies:
            out = features * (1 + gamma) + beta
        """
        x = self.linear(condition, 'film_interaction.cond_mlp.0')
        x = self.leaky_relu(x)
        modulation = self.linear(x, 'film_interaction.cond_mlp.2')  # (B, 2*GSD_DIM)

        gamma, beta = np.split(modulation, 2, axis=-1)
        return features * (1.0 + gamma) + beta

    # --------------------------- Bone interaction -------------------------- #

    def bone_interaction(self, x):
        """
        Bone interaction module.

        Input:
            x: (B, NUM_BONES, D)

        Steps:
            1) transpose to (B, D, NUM_BONES)
            2) mixing MLP along bone dimension:
               Linear -> LeakyReLU -> Linear
            3) residual add and transpose back
        """
        x_T = x.transpose(0, 2, 1)  # (B, D, NUM_BONES)

        h = self.linear(x_T, 'interaction.mixing.0')
        h = self.leaky_relu(h)
        delta = self.linear(h, 'interaction.mixing.2')  # (B, D, NUM_BONES)

        return x + delta.transpose(0, 2, 1)

    # ------------------------------ Forward pass --------------------------- #

    def forward(self, skeletons_data):
        """
        Forward inference.

        Input:
            skeletons_data: (B, 60 + ORI_DIM)
                - 20 joints * 3D positions (flattened)
                - orientation (rot6d) at the end

        Output:
            (B, 3) predicted angles (radian)
        """
        B = skeletons_data.shape[0]

        # 1) Split position and orientation
        ori = skeletons_data[:, -ORI_DIM:]         # (B, ORI_DIM)
        flat_joints = skeletons_data[:, :-ORI_DIM] # (B, 60)

        # 2) Reshape joints and build bone endpoints
        skel = flat_joints.reshape(B, NUM_JOINTS, 3)       # (B, 20, 3)
        parents = skel[:, bone_parents, :]                 # (B, 19, 3)
        children = skel[:, bone_children, :]               # (B, 19, 3)
        bk = np.concatenate([parents, children], axis=-1)  # (B, 19, 6)

        # 3) Positional encodings
        pe_bk = self.position_encoding(bk, PE_FREQ_BK)     # (B, 19, 6 * 2 * 5) = (B, 19, 60)

        eye_nb = np.eye(NUM_BONES, dtype=np.float32)       # (19, 19)
        ok = np.broadcast_to(eye_nb, (B, NUM_BONES, NUM_BONES))  # (B, 19, 19)
        pe_ok = self.position_encoding(ok, PE_FREQ_OK)     # (B, 19, 19 * 2 * 2) = (B, 19, 76)

        # 4) Global Spatial Descriptor (GSD)
        flat = skel.reshape(B, -1)                         # (B, 60)
        g = self.linear(flat, 'gsd_mlp.0')
        g = self.gelu(g)
        g = self.linear(g, 'gsd_mlp.2')
        g = self.gelu(g)
        g_feature = self.linear(g, 'gsd_mlp.4')            # (B, GSD_DIM)

        # 5) FiLM conditioning: modulate GSD using orientation (rot6d)
        g_final = self.film_interaction(g_feature, ori)    # (B, GSD_DIM)
        g_expanded = g_final[:, None, :].repeat(NUM_BONES, axis=1)  # (B, 19, GSD_DIM)

        # 6) Concatenate per-bone features
        # NOTE: Raw orientation embedding is NOT concatenated here in the new architecture.
        oe = np.concatenate([pe_bk, pe_ok, g_expanded], axis=-1)

        # 7) Bone interaction
        oe = self.bone_interaction(oe)

        # 8) Shared backbone
        features = self.linear(oe, 'shared_backbone.0')
        features = self.leaky_relu(features)
        features = self.linear(features, 'shared_backbone.2')
        features = self.leaky_relu(features)               # (B, 19, hidden_dim)

        # 9) Lightweight heads (per-bone)
        out1 = self.linear(features, 'head1')              # (B, 19, 1)
        out2 = self.linear(features, 'head2')              # (B, 19, 1)
        out3 = self.linear(features, 'head3')              # (B, 19, 1)

        o1 = out1.squeeze(-1)                              # (B, 19)
        o2 = out2.squeeze(-1)
        o3 = out3.squeeze(-1)

        # 10) Pooling along bone dimension (Linear(19 -> 1))
        agg1 = self.linear(o1, 'pool1')                    # (B, 1)
        agg2 = self.linear(o2, 'pool2')                    # (B, 1)
        agg3 = self.linear(o3, 'pool3')                    # (B, 1)

        return np.concatenate([agg1, agg2, agg3], axis=-1).astype(np.float32)


# --------------------------------------------------------------------------- #
# ROS node wrapper
# --------------------------------------------------------------------------- #

class InferenceNode(object):
    """
    ROS 1 wrapper for real-time inference and publishing.

    Processing pipeline per message:
        1) Flatten skeleton joints (PoseArray -> 60D)
        2) Append orientation (rot6d)
        3) Forward inference (NumPy)
        4) Temporal smoothing:
            - Butterworth low-pass filter per dimension
            - Exponential moving average (EMA)
        5) Map to final 8-dim output format and publish
    """

    def __init__(self):
        rospy.init_node('skeleton2angle_inference')

        # Load model weights
        try:
            self.model = Skeleton2AngleNumPy(MODEL_WEIGHTS_PATH)
            rospy.loginfo('[OK] Loaded weights from %s', MODEL_WEIGHTS_PATH)
        except Exception as e:
            rospy.logerr('Model load failed: %s', e)
            rospy.signal_shutdown('Cannot continue without weights')
            return

        # ---------------------- Temporal filter configuration ---------------------- #
        # Butterworth low-pass filter (2nd-order IIR)
        # - Designed for 60 Hz callback rate
        # - Removes high-frequency jitter while keeping response reasonably fast
        order = 2
        fs = 60.0   # expected callback frequency (Hz)
        fc = 10.0   # cutoff frequency (Hz)
        nyq = 0.5 * fs
        normal_cut = fc / nyq

        self.b, self.a = butter(order, normal_cut, btype='low', analog=False)

        # Initialize per-dimension IIR filter state
        zi_base = lfilter_zi(self.b, self.a)
        self.zi_filter = [zi_base * 0.0 for _ in range(3)]

        # EMA smoothing for additional stabilization
        self.ema_alpha = 0.1
        self.ema = None

        # ------------------------------ ROS I/O ----------------------------------- #
        self.pub = rospy.Publisher(OUTPUT_TOPIC, Float32MultiArray, queue_size=1)
        self.pub_data = rospy.Publisher('/model_out_data', Float32MultiArray, queue_size=1)

        rospy.Subscriber(INPUT_TOPIC, HandSyncData, self.callback)
        rospy.loginfo('Node ready - waiting for %s', INPUT_TOPIC)

    def _flatten_posearray(self, msg):
        """
        Flatten PoseArray positions into a 60D vector.

        - Skips joints whose indices are listed in KEYWORDS.
        - Uses only position (x, y, z).
        """
        data = []
        for idx, p in enumerate(msg.pose_array.poses):
            if idx in KEYWORDS:
                continue
            data.extend([p.position.x, p.position.y, p.position.z])
        return np.asarray(data, np.float32)

    def callback(self, msg):
        """
        Per-message inference callback.

        Steps:
            1) Build model input (1, 60 + ORI_DIM)
            2) Run forward inference -> (1, 3) in radian
            3) Apply LPF + EMA
            4) Overwrite final output indices [1:4] with predicted angles in degrees
            5) Publish both standard output and debug output
        """
        try:
            # 1) Flatten joints: (60,) -> (1, 60)
            x_flat = self._flatten_posearray(msg)[None, :]

            # 2) Orientation input (rot6d): last ORI_DIM elements
            extra = np.array(msg.angles[-ORI_DIM:], dtype=np.float32).reshape(1, ORI_DIM)

            # 3) Concatenate into model input: (1, 60 + ORI_DIM)
            x_input = np.concatenate([x_flat, extra], axis=1)

            expected_len = NUM_JOINTS * 3 + ORI_DIM
            if x_input.shape[1] != expected_len:
                rospy.logwarn('Unexpected input length %d (expected %d)', x_input.shape[1], expected_len)
                return

            # 4) Forward inference
            out = self.model.forward(x_input)  # (1, 3) radian
            out_vec = out.flatten()            # (3,)

            # 5) Butterworth LPF per dimension
            filtered = np.zeros_like(out_vec)
            for i in range(3):
                y, self.zi_filter[i] = lfilter(self.b, self.a, [out_vec[i]], zi=self.zi_filter[i])
                filtered[i] = y[0]

            # 6) EMA smoothing
            if self.ema is None:
                self.ema = filtered.copy()
            else:
                self.ema = self.ema_alpha * filtered + (1.0 - self.ema_alpha) * self.ema

            # 7) Construct final 8D output:
            # - Start from msg.angles without the orientation tail
            # - Overwrite indices 1:4 using the model prediction (degrees)
            final_out = np.array(msg.angles[:-ORI_DIM], dtype=np.float32).reshape(8)
            final_out[1:4] = self.ema * radTodeg

            # 8) Debug output includes orientation appended
            data_out = np.concatenate([final_out, extra.squeeze()], axis=0)

            self.pub.publish(Float32MultiArray(data=final_out.tolist()))
            self.pub_data.publish(Float32MultiArray(data=data_out.tolist()))

        except Exception as e:
            rospy.logerr('Inference error: %s', e)

    def spin(self):
        """ROS spin loop."""
        rospy.spin()


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

if __name__ == '__main__':
    node = InferenceNode()
    if not rospy.is_shutdown():
        node.spin()
