#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed to enable 3D)

class PoseArrayPlotter(object):
    def __init__(self):
        # Parameters
        # Axes range; adjust if your environment is larger/smaller
        self.xlim = rospy.get_param("~xlim", [-0.5, 0.5])
        self.ylim = rospy.get_param("~ylim", [-0.5, 0.5])
        self.zlim = rospy.get_param("~zlim", [-0.5, 0.5])
        # Image size
        self.img_width = rospy.get_param("~img_width", 1024)
        self.img_height = rospy.get_param("~img_height", 768)

        # State
        self.latest_xyz = None
        self.bridge = CvBridge()

        # ROS I/O
        self.sub = rospy.Subscriber("rviz", PoseArray, self.pose_cb, queue_size=1)
        self.pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)

        # Matplotlib Figure preparation
        self.fig = Figure(figsize=(self.img_width / 100.0, self.img_height / 100.0), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')

        rospy.loginfo("PoseArrayPlotter initialized: subscribing rviz, publishing camera/image_raw")

    def pose_cb(self, msg):
        """Callback for PoseArray messages: update and publish immediately."""
        if len(msg.poses) == 0:
            return
        p0 = msg.poses[0].position
        self.latest_xyz = (-p0.x, - p0.z-1, p0.y)

        # Prepare axes
        self.ax.clear()
        self.ax.set_xlim(self.xlim[0], self.xlim[1])
        self.ax.set_ylim(self.ylim[0], self.ylim[1])
        self.ax.set_zlim(self.zlim[0], self.zlim[1])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("First Pose Position (3D)")

        # Draw coordinate axes
        self.ax.plot([self.xlim[0], self.xlim[1]], [0, 0], [0, 0], linewidth=1)
        self.ax.plot([0, 0], [self.ylim[0], self.ylim[1]], [0, 0], linewidth=1)
        self.ax.plot([0, 0], [0, 0], [self.zlim[0], self.zlim[1]], linewidth=1)

        # 13 335 168
        target = np.array([-0.012, -0.645, -0.06])


        # Plot latest point
        x, y, z = self.latest_xyz
        point = np.array([x,y,z])
        distance = np.linalg.norm(point - target)
        threshold = 0.05
        color = 'green' if distance <= threshold else 'red'

        self.ax.scatter([x], [y], [z], s=50, color=color)
        self.ax.scatter([-0.012], [-0.645], [-0.06], s=50, color='blue')
        
        self.ax.text(x, y, z, 
                    "x={:.3f}, y={:.3f}, z={:.3f}".format(x, y, z),
                    fontsize=9, color='blue')


        self.fig.tight_layout()
        self.canvas.draw()
        width, height = self.fig.canvas.get_width_height()
        buf = np.frombuffer(self.canvas.tostring_rgb(), dtype=np.uint8)
        img_rgb = buf.reshape((height, width, 3))
        


        # Convert to BGR for OpenCV then to ROS Image
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        img_bgr = cv2.flip(img_bgr, 0)
        img_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()

        self.pub.publish(img_msg)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("pose_array_plotter")
    node = PoseArrayPlotter()
    node.spin()
