#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse

# Global variable to store the latest angle array received from /angle topic
latest_angles = None
# List to store recorded reference points (each is an entire angle array) - maximum 3 points
recorded_points = []

def angle_callback(msg):
    """
    Callback for /angle topic.
    Stores the entire array from the Float32MultiArray message as the current angle data.
    """
    global latest_angles
    if msg.data:
        # Copy the entire array
        latest_angles = list(msg.data)
        rospy.loginfo("Received angle array: %s", latest_angles)
    else:
        rospy.logwarn("Received an empty angle array.")

def record_point_callback(req):
    """
    Service callback:
    Records the current angle array as a reference point and saves the updated list to the ROS Parameter Server.
    """
    global latest_angles, recorded_points
    if latest_angles is None:
        return TriggerResponse(success=False, message="No angle data received yet.")

    if len(recorded_points) >= 3:
        return TriggerResponse(success=False, message="Maximum of 3 reference points have already been recorded.")

    samples = []
    sample_count = 20   # Number of samples to average.
    rate = rospy.Rate(20)  # Sampling at 20 Hz.
    
    # Collect 20 samples.
    while len(samples) < sample_count:
        if latest_angles is not None:
            samples.append(list(latest_angles))
            print([round(x, 6) for x in latest_angles])
            rospy.loginfo("Collected sample %d: %s", len(samples), latest_angles)
        rate.sleep()
    
    # Compute element-wise average across the collected samples.
    # zip(*samples) aggregates elements at the same index from all samples.
    averaged_angles = [sum(angles) / len(angles) for angles in zip(*samples)]
    recorded_points.append(averaged_angles)
    print("Calibration {0} : ".format(len(recorded_points)),[round(x, 6) for x in averaged_angles])

    rospy.loginfo("Recorded point %d: %s", len(recorded_points), averaged_angles)
    
    # Save calibration points to the ROS parameter server under key 'calibration/recorded_points'
    rospy.set_param('calibration/recorded_points', recorded_points)
    
    response_message = "Reference point recorded successfully. Total recorded points: {0}".format(len(recorded_points))
    if len(recorded_points) == 3:
        rospy.loginfo("All recorded calibration points: %s", recorded_points)
        response_message += " (All calibration points recorded: " + str(recorded_points) + ")"
    
    return TriggerResponse(success=True, message=response_message)

def main():
    """
    Main function to initialize the node, subscribe to /angle topic, and create the record_point service.
    """
    rospy.init_node('point_recorder', anonymous=False)
    
    # Subscribe to /angle topic, expected message type is Float32MultiArray
    rospy.Subscriber('/raw_hand_angles', Float32MultiArray, angle_callback)
    
    # Create a service server with the Trigger service type to record a calibration point on demand
    rospy.Service('record_point', Trigger, record_point_callback)
    
    rospy.loginfo("Point recorder node started. Waiting for angle data and service calls to record calibration points.")
    rospy.spin()

if __name__ == '__main__':
    main()

    # enter at ternimal -> rosservice call /record_point "{}"
    # 1 is init, 2 is extend 3 is grap
    # TODO : add more cali points

