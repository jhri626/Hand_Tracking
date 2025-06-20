#!/usr/bin/env python
"""
ROS1 node for FingerAngleFFN inference using NumPy weights loaded from .npz file.
Simplified: hardcoded configuration without ROS parameter server.
Compatible with Python 2.7.
Subscribe to geometry_msgs/PoseArray, extract input features, perform inference, load scaler via joblib if available, or fallback to manual denormalization parameters.
Edit MODEL_WEIGHTS_PATH, LAYERNORM_NAMES, LINEAR_NAMES, and input mapping as needed.
"""
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray
from scipy.special import erf

# === Configuration: Edit these values as needed ===
MODEL_WEIGHTS_PATH = r'C:/Users/dyros/Desktop/dummy_ws/model/model_weights.npz'  # Path to .npz file
SCALER_PATH = r'C:/Users/dyros/Desktop/dummy_ws/model/y_scaler.pkl'  # Path to joblib-saved scaler
# Specify LayerNorm and Linear layer names manually based on loaded npz keys
LAYERNORM_NAMES = [
    'ffn_blocks.0.block.0',
    'ffn_blocks.1.block.0',
    'ffn_blocks.2.block.0'
]
LINEAR_NAMES = [
    'embedding_layer',
    'ffn_blocks.0.block.1', 'ffn_blocks.0.block.3',
    'ffn_blocks.1.block.1', 'ffn_blocks.1.block.3',
    'ffn_blocks.2.block.1', 'ffn_blocks.2.block.3',
    'output_layer'
]
EPS = 1e-5
INPUT_TOPIC = '/hand_joints'      # Input topic name (PoseArray)
OUTPUT_TOPIC = '/model_out'        # Output topic name (Float32MultiArray)
KEYWORDS = [5,10,15,20,21,22,23,24,25]

# Optional fallback manual denormalization parameters
min_ = np.array([0.6937413,  0.60397506, 0.45551434, 0.20958792, 0.97614574, 0.88981766,
                 0.8967024,  0.8895251], dtype=np.float32)
scale_ = np.array([0.0097228,  0.0163719,  0.01707425, 0.01736045, 0.00853814, 0.00489475,
                   0.00491007, 0.00486718], dtype=np.float32)

class FingerAngleFFNNumPy:
    """
    NumPy-based implementation of the FingerAngleFFN model for inference.
    LayerNorm and Linear layer names are specified via LAYERNORM_NAMES and LINEAR_NAMES.
    """
    def __init__(self, npz_path, layernorm_names, linear_names, eps=1e-5):
        try:
            loaded = np.load(npz_path)
        except Exception as e:
            raise IOError('Failed to load npz file {}: {}'.format(npz_path, e))
        self.params = {k: loaded[k].astype(np.float32) for k in loaded.files}
        self.ln_names = layernorm_names
        self.lin_names = linear_names
        self.eps = eps
        self.op_sequence = []
        prev_linear = None
        # Build sequence: embedding, blocks, output
        if 'embedding_layer' in self.lin_names:
            self.op_sequence.append(('linear', 'embedding_layer'))
            prev_linear = 'embedding_layer'
        num_blocks = len(self.ln_names)
        for i in range(num_blocks):
            ln_name = 'ffn_blocks.{}.block.0'.format(i)
            lin1 = 'ffn_blocks.{}.block.1'.format(i)
            lin2 = 'ffn_blocks.{}.block.3'.format(i)
            if ln_name in self.ln_names:
                self.op_sequence.append(('layernorm', ln_name))
                prev_linear = None
            if lin1 in self.lin_names:
                if prev_linear is not None:
                    self.op_sequence.append(('gelu', None))
                self.op_sequence.append(('linear', lin1))
                prev_linear = lin1
            if lin2 in self.lin_names:
                if prev_linear is not None:
                    self.op_sequence.append(('gelu', None))
                self.op_sequence.append(('linear', lin2))
                prev_linear = lin2
        if 'output_layer' in self.lin_names:
            self.op_sequence.append(('linear', 'output_layer'))

    def layernorm(self, x, name):
        gamma = self.params.get(name + '.weight')
        beta = self.params.get(name + '.bias')
        if gamma is None or beta is None:
            raise KeyError('LayerNorm parameters for {} not found'.format(name))
        mean = np.mean(x, axis=-1, keepdims=True)
        var = np.mean((x - mean) ** 2, axis=-1, keepdims=True)
        x_norm = (x - mean) / np.sqrt(var + self.eps)
        return x_norm * gamma + beta

    def linear(self, x, name):
        W = self.params.get(name + '.weight')
        if W is None:
            raise KeyError('Linear weight for {} not found'.format(name))
        b = self.params.get(name + '.bias', np.zeros(W.shape[0], dtype=np.float32))
        return x.dot(W.T) + b

    def gelu(self, x):
        return 0.5 * x * (1.0 + erf(x / np.sqrt(2.0)))

    def forward(self, x):
        x = x.astype(np.float32)
        out = x
        for op, name in self.op_sequence:
            if op == 'layernorm':
                out = self.layernorm(out, name)
            elif op == 'linear':
                out = self.linear(out, name)
            elif op == 'gelu':
                out = self.gelu(out)
        return out

class InferenceNode:
    """
    ROS1 node to subscribe to PoseArray topic, perform inference, load scaler via joblib, fallback manual denormalization, and publish.
    Simplified: no parameter server; uses hardcoded configuration above.
    Compatible with Python 2.7.
    """
    def __init__(self):
        rospy.init_node('finger_angle_inference')
        try:
            self.model = FingerAngleFFNNumPy(MODEL_WEIGHTS_PATH, LAYERNORM_NAMES, LINEAR_NAMES, EPS)
            rospy.loginfo('Loaded model weights from {}'.format(MODEL_WEIGHTS_PATH))
        except Exception as e:
            rospy.logerr('Failed to load model weights: {}'.format(e))
            rospy.signal_shutdown('Model load failure')
            return
        # Attempt to load scaler via joblib
        self.scaler = None
        try:
            try:
                import joblib
            except ImportError:
                from sklearn.externals import joblib
            self.scaler = joblib.load(SCALER_PATH)
            rospy.loginfo('Loaded scaler via joblib from {}'.format(SCALER_PATH))
        except Exception as e:
            rospy.logwarn('Scaler load via joblib failed: {}'.format(e))
            rospy.logwarn('Fallback to manual denormalization parameters if applicable.')
        self.pub = rospy.Publisher(OUTPUT_TOPIC, Float32MultiArray, queue_size=1)
        rospy.Subscriber(INPUT_TOPIC, PoseArray, self.callback)
        rospy.loginfo('Inference node initialized. Waiting for PoseArray input...')

    def callback(self, msg):
        try:
            poses = msg.poses
            data_list = []
            for idx,p in enumerate(poses):

                if idx in KEYWORDS:
                    continue
                else:
                    data_list.extend([
                        p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
                    ])
            data = np.array(data_list, dtype=np.float32).reshape(1, -1)
            output_norm = self.model.forward(data)
            # Denormalize
            if self.scaler is not None:
                try:
                    output_denorm = self.scaler.inverse_transform(output_norm)
                except Exception as e_inv:
                    rospy.logwarn('Scaler inverse_transform failed: {}'.format(e_inv))
                    output_denorm = output_norm
            else:
                # Manual denormalization: (X_norm - min_) / scale_
                if output_norm.shape[1] == min_.shape[0]:
                    try:
                        output_denorm = (output_norm - min_) / scale_
                    except Exception as e_den:
                        rospy.logwarn('Manual denormalization failed: {}'.format(e_den))
                        output_denorm = output_norm
                else:
                    rospy.logwarn('Output dimension {} does not match manual denorm params {}'.format(output_norm.shape[1], min_.shape[0]))
                    output_denorm = output_norm
            out_msg = Float32MultiArray()
            out_msg.data = output_denorm.flatten().tolist()
            self.pub.publish(out_msg)
        except Exception as e:
            rospy.logerr('Inference error: {}'.format(e))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = InferenceNode()
    if not rospy.is_shutdown():
        node.spin()
