#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from vr.msg import HandSyncData

import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

class PoseArrayPlotter(object):
    def __init__(self):
        # Parameters
        self.xlim = rospy.get_param("~xlim", [-0.5, 0.5])
        self.ylim = rospy.get_param("~ylim", [-0.5, 0.5])
        self.zlim = rospy.get_param("~zlim", [-0.5, 0.5])
        self.img_width = rospy.get_param("~img_width", 1024)
        self.img_height = rospy.get_param("~img_height", 768)

        # State
        self.latest_xyz = None
        self.bridge = CvBridge()

        # Variables to store the latest angles
        self.latest_raw_angles = None
        self.latest_model_angles = None

        # ROS I/O
        self.sub = rospy.Subscriber("rviz", PoseArray, self.pose_cb, queue_size=1)
        self.pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)

        self.raw_sub = rospy.Subscriber("hand_sync_data", HandSyncData, self.raw_cb, queue_size=1)
        self.model_sub = rospy.Subscriber("model_out_data", Float32MultiArray, self.model_cb, queue_size=1) 

        # Matplotlib Figure preparation
        self.fig = Figure(figsize=(self.img_width / 100.0, self.img_height / 100.0), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')

        rospy.loginfo("PoseArrayPlotter initialized.")

    def raw_cb(self, msg):
        temp = np.array(msg.angles, dtype=np.float32)
        if len(temp) >= 4:
            self.latest_raw_angles = temp[1:4]
        else:
            self.latest_raw_angles = temp 

    def model_cb(self, msg):
        temp = np.array(msg.data, dtype=np.float32)
        if len(temp) >= 4:
            self.latest_model_angles = temp[1:4]
        else:
            self.latest_model_angles = temp 

    def pose_cb(self, msg):
        if len(msg.poses) == 0:
            return
        
        # 1. 3D Plotting using Matplotlib
        p0 = msg.poses[0].position
        self.latest_xyz = (-p0.x, -p0.z-1, p0.y)

        self.ax.clear()
        self.ax.set_xlim(self.xlim[0], self.xlim[1])
        self.ax.set_ylim(self.ylim[0], self.ylim[1])
        self.ax.set_zlim(self.zlim[0], self.zlim[1])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("First Pose Position (3D)")

        self.ax.plot([self.xlim[0], self.xlim[1]], [0, 0], [0, 0], linewidth=1)
        self.ax.plot([0, 0], [self.ylim[0], self.ylim[1]], [0, 0], linewidth=1)
        self.ax.plot([0, 0], [0, 0], [self.zlim[0], self.zlim[1]], linewidth=1)

        target = np.array([-0.012, -0.645, -0.06])
        x, y, z = self.latest_xyz
        
        dist_pos = np.linalg.norm(np.array([x,y,z]) - target)
        color = 'green' if dist_pos <= 0.05 else 'red'
        
        self.ax.scatter([x], [y], [z], s=50, color=color)
        self.ax.scatter(target[0], target[1], target[2], s=50, color='blue')
        self.ax.text(x, y, z, "x={:.3f}, y={:.3f}, z={:.3f}".format(x, y, z), fontsize=9, color='blue')

        self.fig.tight_layout()
        self.canvas.draw()
        
        # 2. Convert Matplotlib figure to OpenCV Image (RGB -> BGR)
        width, height = self.fig.canvas.get_width_height()
        buf = np.frombuffer(self.canvas.tostring_rgb(), dtype=np.uint8)
        img_rgb = buf.reshape((height, width, 3))
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

        
        is_similar = False
        
        if self.latest_raw_angles is not None and self.latest_model_angles is not None:
            min_len = min(len(self.latest_raw_angles), len(self.latest_model_angles))
            raw_vec = self.latest_raw_angles[:min_len]
            model_vec = self.latest_model_angles[:min_len]

            error_val = np.linalg.norm(raw_vec - model_vec)
            if error_val < 5:
                is_similar = True

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            line_step = 30
            
            # Target position after flip: x=20, y=40 (Top-Left)
            # Source position before flip: x = Width - 20, y = Height - 40
            # Note: putText origin is bottom-left of the text string.
            
            # Let's align text to the Right side of the pre-flipped image 
            # so it becomes Left side of the flipped image.
            
            # Start coordinates (Pre-Flip: Bottom-Right area)
            start_x = width - 250  # Adjust based on text length
            start_y = height - 100

            # (1) Raw Values
            if len(raw_vec) >= 3:
                raw_str = "Raw: {:.2f}, {:.2f}, {:.2f}".format(raw_vec[0], raw_vec[1], raw_vec[2])
            else:
                raw_str = "Raw: " + str(raw_vec)
            
            # Draw at (start_x, start_y)
            cv2.putText(img_bgr, raw_str, (start_x, start_y), font, font_scale, (0, 180, 0), thickness)

            # (2) Model Values
            if len(model_vec) >= 3:
                mod_str = "Mod: {:.2f}, {:.2f}, {:.2f}".format(model_vec[0], model_vec[1], model_vec[2])
            else:
                mod_str = "Mod: " + str(model_vec)
            
            # Move DOWN in pre-flip (which is UP in post-flip) -> No, flip(-1) inverts Y too.
            # (Height - 40) -> becomes (40).
            # (Height - 70) -> becomes (70).
            # So to go DOWN in the final image, we must go UP (subtract Y) in pre-flip image?
            # Let's tracing:
            # Point (W-20, H-40) -> Flip(-1) -> (20, 40).
            # Point (W-20, H-70) -> Flip(-1) -> (20, 70).
            # So simply subtract more from Height.
            
            cv2.putText(img_bgr, mod_str, (start_x, start_y - line_step), font, font_scale, (0, 200, 255), thickness)

            # (3) Diff Value
            err_str = "Diff: {:.3f}".format(error_val)
            cv2.putText(img_bgr, err_str, (start_x, start_y - line_step * 2), font, font_scale, (0, 0, 255), thickness)

        # Draw Match Box (Target: Top-Right after flip)
        # Pre-flip: Bottom-Left.
        if is_similar:
            # Final Target: x ~ Width-60, y ~ 10
            # Pre-flip Source: x ~ 60, y ~ Height-10
            
            pt1 = (60, height - 10)
            pt2 = (10, height - 60)
            cv2.rectangle(img_bgr, pt1, pt2, (255, 0, 0), -1)
            
            cv2.putText(img_bgr, "MATCH", (15, height - 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # ---------------------------------------------------------
        # 4. Flip Everything (-1)
        # This fixes the Plot (as you said) AND now rotates the text 
        # so it becomes readable if the monitor is upside down.
        # ---------------------------------------------------------
        img_bgr = cv2.flip(img_bgr, 0) 

        # 5. Publish ROS Image
        img_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        self.pub.publish(img_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("pose_array_plotter")
    node = PoseArrayPlotter()
    node.spin()