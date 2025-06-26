#!/usr/bin/env python
"""
ROS 1 node for Skeleton2Mesh-based hand-angle inference (NumPy only).

Flow:
  PoseArray -> NumPy model (Skeleton2AngleNumPy) -> Float32MultiArray
* No normalisation / denormalisation step.
* Compatible with Python 2.7 (ROS Kinetic/Melodic).
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
from scipy.special import erf
from bone import bone_parents, bone_children   # length 19
from vr.msg import HandSyncData

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
MODEL_WEIGHTS_PATH = r'C:/Users/dyros/Desktop/dummy_ws/model/model_weights.npz'

NUM_JOINTS = 20
NUM_BONES  = 19
PE_FREQ_BK = 5
PE_FREQ_OK = 2
GSD_DIM    = 100
NEG_SLOPE  = 0.01                 # LeakyReLU slope

INPUT_TOPIC  = '/hand_sync_data'
OUTPUT_TOPIC = '/model_out'
KEYWORDS     = [0, 21, 22, 23, 24, 25]   # joints to skip

# --------------------------------------------------------------------------- #
# NumPy model (Skeleton2Mesh replica)
# --------------------------------------------------------------------------- #
class Skeleton2AngleNumPy(object):
    """Pure-NumPy implementation reproducing the PyTorch Skeleton2Mesh forward."""
    def __init__(self, npz_path):
        self.params = {k: v.astype(np.float32) for k, v in np.load(npz_path).items()}

    # --------------------------------------------------------------------- #
    # Basic ops
    # --------------------------------------------------------------------- #
    @staticmethod
    def gelu(x):
        return 0.5 * x * (1. + erf(x / np.sqrt(2., dtype=x.dtype)))

    @staticmethod
    def leaky_relu(x, neg_slope=NEG_SLOPE):
        return np.where(x >= 0, x, neg_slope * x)

    def linear(self, x, prefix):
        """x: (..., in_features) -> (..., out_features)"""
        w = self.params[prefix + '.weight']            # (out, in)
        b = self.params[prefix + '.bias']
        return np.dot(x, w.T) + b

    # --------------------------------------------------------------------- #
    # Positional encoding
    # --------------------------------------------------------------------- #
    @staticmethod
    def position_encoding(x, num_freqs):
        """
        x: ndarray [..., D]
        returns ndarray [..., D * 2 * num_freqs] (sin|cos concatenation)
        """
        freqs = (2.0 ** np.arange(num_freqs, dtype=x.dtype)) * np.pi   # (L,)
        x_exp = x[..., None] * freqs                                   # [..., D, L]
        sin   = np.sin(x_exp)
        cos   = np.cos(x_exp)
        pe    = np.concatenate([sin, cos], axis=-1)                    # [..., D, 2L]
        new_shape = x.shape[:-1] + (-1,)
        return pe.reshape(new_shape)                           # flatten

    # --------------------------------------------------------------------- #
    # Forward pass
    # --------------------------------------------------------------------- #
    def forward(self, skeletons_data):
        """
        Input : skeletons_flat (B, 64) - 20 joints * 3 coords + 4 quaternion.
        Output: (B, 8) - angle vector (no scaling applied).
        """
        B = skeletons_data.shape[0]

        ori = skeletons_data[:, -3:]                     # (B,4)
        flat_joints = skeletons_data[:, :-3]             # (B,60)

        # 1 | reshape joints
        skel = flat_joints.reshape(B, NUM_JOINTS, 3)                # (B,20,3)

        # 2 | bone endpoints
        parents  = skel[:, bone_parents, :]                            # (B,19,3)
        children = skel[:, bone_children, :]
        Bk = np.concatenate([parents, children], axis=-1)              # (B,19,6)

        # 3 | positional encodings
        pe_bk = self.position_encoding(Bk, PE_FREQ_BK)                 # (B,19,60)

        eye_nb = np.eye(NUM_BONES, dtype=np.float32)                   # (19,19)
        ok     = np.broadcast_to(eye_nb, (B, NUM_BONES, NUM_BONES))    # (B,19,19)
        pe_ok  = self.position_encoding(ok, PE_FREQ_OK)                # (B,19,76)

        # 4 | global spatial descriptor g
        flat = skel.reshape(B, -1)                                     # (B,60)
        g    = self.linear(flat, 'gsd_mlp.0')
        g    = self.gelu(g)
        g    = self.linear(g, 'gsd_mlp.2')
        g    = self.gelu(g)
        g    = self.linear(g, 'gsd_mlp.4')                             # (B,100)
        g    = g[:, None, :].repeat(NUM_BONES, axis=1)                 # (B,19,100)

        oe = np.dot(ori, self.params['orientation_embed.weight'].T) + self.params['orientation_embed.bias']
        oe = oe[:, None, :].repeat(NUM_BONES, axis=1)      # (B,19,3)


        # 5 | OE feature
        OE = np.concatenate([pe_bk, pe_ok, g, oe], axis=-1)

        
        # --- four heads -------------------------------------------------- #
        def head(OE, name):                                            # -> (B,19,C)
            h = self.linear(OE, name + '.0')
            h = self.leaky_relu(h)
            h = self.linear(h, name + '.2')
            h = self.leaky_relu(h)
            h = self.linear(h, name + '.4')
            return h

        out1 = head(OE, 'head1')                                       # (B,19,1)
        out2 = head(OE, 'head2')                                       # (B,19,1)
        out3 = head(OE, 'head3')                                       # (B,19,1)
        

        # --- pooling along bone dim ------------------------------------- #
        def pool(x, pool_name):                                        # (B,C,19) or (B,19)
            w = self.params[pool_name + '.weight']                     # (1,19)
            b = self.params[pool_name + '.bias'][0]                    # scalar
            return (x * w).sum(axis=-1) + b                            # (B,C) or (B,)

        # out1 / out2: transpose bones last -> last dim
        o1 = out1.squeeze(-1)                                          # (B,19)
        o2 = out2.squeeze(-1)                                          # (B,19)

        # out3 / out4: squeeze last singleton
        o3 = out3.squeeze(-1)                                          # (B,19)
        

        agg1 = pool(o1, 'pool1')[:, None]                              # (B,1)
        agg2 = pool(o2, 'pool2')[:, None]
        agg3 = pool(o3, 'pool3')[:, None]

        # concat -> (B,8)
        return np.concatenate([agg1, agg2, agg3], axis=-1).astype(np.float32)

# --------------------------------------------------------------------------- #
# ROS node wrapper
# --------------------------------------------------------------------------- #
class InferenceNode(object):
    """ROS 1 node that embeds Skeleton2AngleNumPy and publishes 8-D predictions."""
    def __init__(self):
        rospy.init_node('skeleton2angle_inference')

        # Load model
        try:
            self.model = Skeleton2AngleNumPy(MODEL_WEIGHTS_PATH)
            rospy.loginfo('[OK] Loaded weights from %s', MODEL_WEIGHTS_PATH)
        except Exception as e:
            rospy.logerr('Model load failed: %s', e)
            rospy.signal_shutdown('Cannot continue without weights')
            return
        
        # Exponential Moving Average parameters
        # smoothing factor alpha: higher -> output tracks new values more closely
        self.ema_alpha = 0.5
        self.ema       = None  # stores previous EMA value, shape (1,8)

        # ROS I/O
        self.pub = rospy.Publisher(OUTPUT_TOPIC, Float32MultiArray, queue_size=1)
        rospy.Subscriber(INPUT_TOPIC, HandSyncData, self.callback)
        rospy.loginfo('Node ready - waiting for %s', INPUT_TOPIC)

    # --------------------------------------------------------------------- #
    def _flatten_posearray(self, msg):
        """PoseArray -> flat list of 60 floats (skip KEYWORDS joints)."""
        data = []
        # ori = msg.poses[0].orientation
        # data.extend([ori.x, ori.y, ori.z, ori.w])

        for idx, p in enumerate(msg.poses):
            if idx in KEYWORDS:
                continue
            data.extend([
                p.position.x, p.position.y, p.position.z
            ])

            # data.extend([p.position.x, p.position.y, p.position.z,
                        #  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            # data.extend([p.position.x, p.position.y, p.position.z])
        return np.asarray(data, np.float32)

    # --------------------------------------------------------------------- #
    def callback(self, msg):
        try:
            x_flat = self._flatten_posearray(msg.pose_array)[None, :]             # (1,60)
            extra = np.array(msg.angles[-3:], dtype=np.float32).reshape(1, 3)
            x_flat = np.concatenate([x_flat, extra], axis=1)


            if x_flat.shape[1] != NUM_JOINTS * 3 + 3:
                rospy.logwarn('Unexpected input length %d', x_flat.shape[1])
                return

            out = self.model.forward(x_flat)                           # (1,3)
            final_out = np.array(msg.angles[:-3], dtype=np.float32).reshape(8)
            final_out[1:4] = 0. * final_out[1:4] + 1.0 * out.reshape(3)
            
            
            
            if self.ema is None:
                self.ema = final_out.copy()
            else:
                self.ema = self.ema_alpha * final_out + (1.0 - self.ema_alpha) * self.ema

            ema_list = self.ema.flatten().tolist()
            
            self.pub.publish(Float32MultiArray(data=ema_list))


        except Exception as e:
            rospy.logerr('Inference error: %s', e)

    def spin(self):
        rospy.spin()

# --------------------------------------------------------------------------- #
if __name__ == '__main__':
    node = InferenceNode()
    if not rospy.is_shutdown():
        node.spin()
