#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from collections import deque

# 1. Buffer settings
buffer_size = 100
time_buffer = deque([0.0] * buffer_size, maxlen=buffer_size)

# Two data buffers for the two array elements
data1_buffer = deque([0.0] * buffer_size, maxlen=buffer_size)
data2_buffer = deque([0.0] * buffer_size, maxlen=buffer_size)
data3_buffer = deque([0.0] * buffer_size, maxlen=buffer_size)
data4_buffer = deque([0.0] * buffer_size, maxlen=buffer_size)

start_time = None

def topic_callback(msg):
    """
    Callback for a Float32MultiArray topic.
    Appends the first two elements of msg.data along with a timestamp.
    If fewer than two elements are present, missing values default to 0.0.
    """
    global start_time
    if start_time is None:
        start_time = rospy.get_time()
    t = rospy.get_time() - start_time

    # Extract first two elements, defaulting to 0.0 if not present
    data1 = msg.data[0] if len(msg.data) > 0 else 0.0
    data2 = msg.data[1] if len(msg.data) > 1 else 0.0
    data3 = msg.data[2] if len(msg.data) > 2 else 0.0
    data4 = msg.data[3] if len(msg.data) > 3 else 0.0

    time_buffer.append(t)
    data1_buffer.append(data1)
    data2_buffer.append(data2)
    data3_buffer.append(data3)
    data4_buffer.append(data4)

if __name__ == '__main__':
    # 2. Initialize the ROS node
    rospy.init_node('realtime_two_element_plot')

    # 3. Subscribe to the Float32MultiArray topic
    topic_name = '/data'  # Replace with your actual topic
    rospy.Subscriber(topic_name, Float32MultiArray, topic_callback)

    # 4. Set up Matplotlib in interactive mode
    plt.ion()
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], lw=2, label='euler')
    line2, = ax.plot([], [], lw=2, linestyle='--', label='geo')
    line3, = ax.plot([], [], lw=2, linestyle='--', label='geo')
    line4, = ax.plot([], [], lw=2, linestyle='--', label='ik')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Value')
    ax.set_title('Real-time plot thumb joint angle')
    ax.legend(loc='upper right')
    ax.set_ylim(-180, 180)  # Adjust based on data range

    # 5. Main loop: update plot at 10 Hz
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if time_buffer:
            # Update each line with its buffer
            line1.set_data(time_buffer, data1_buffer)
            line2.set_data(time_buffer, data2_buffer)
            line3.set_data(time_buffer, data3_buffer)
            line4.set_data(time_buffer, data4_buffer)
            # Adjust x-axis to show only the current window
            time_padding = 2
            ax.set_xlim(time_buffer[0], time_buffer[-1]+time_padding)
            # Redraw canvas
            fig.canvas.draw()
            fig.canvas.flush_events()

        rate.sleep()
