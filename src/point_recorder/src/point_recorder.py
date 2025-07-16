#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse

class PointRecorder:
    """
    PointRecorder node:
    - Subscribes to '/model_out' topic to receive angle arrays.
    - Provides a Trigger service 'record_point' to record averaged reference points.
    - Records up to 5 reference points and stores them on the ROS Parameter Server under 'calibration/recorded_points'.
    """

    def __init__(self):
        # Hardcoded configurations for Python 2.7 compatibility
        self.topic_name = '/model_out'
        self.service_name = 'record_point'
        self.sample_count = 20
        self.sample_rate_hz = 20
        self.max_points = 5
        self.param_key = 'calibration/recorded_points'

        # Internal state
        self.latest_angles = None
        self.recorded_points = []

        # Subscriber for angle data\        
        self.subscriber = rospy.Subscriber(
            self.topic_name, Float32MultiArray, self.angle_callback
        )
        # Service to record a calibration point
        self.service = rospy.Service(
            self.service_name, Trigger, self.handle_record_point
        )

        rospy.loginfo(
            "PointRecorder initialized: topic='%s', service='%s', sample_count=%d, max_points=%d",
            self.topic_name, self.service_name, self.sample_count, self.max_points
        )

    def angle_callback(self, msg):
        """
        Callback for '/model_out' topic.
        Stores the latest incoming angle array.
        """
        if not msg.data:
            rospy.logwarn("Received empty angle array on '%s'.", self.topic_name)
            return

        # Copy data to avoid aliasing issues
        self.latest_angles = list(msg.data)
        rospy.loginfo("Updated latest_angles: %s", self.latest_angles)

    def handle_record_point(self, req):
        """
        Service callback to record an averaged angle array as a new reference point.
        """
        if self.latest_angles is None:
            return TriggerResponse(success=False, message="No angle data received yet.")

        if len(self.recorded_points) >= self.max_points:
            msg = "Maximum of %d reference points reached." % self.max_points
            return TriggerResponse(success=False, message=msg)

        samples = []
        rate = rospy.Rate(self.sample_rate_hz)

        rospy.loginfo("Collecting %d samples at %d Hz.", self.sample_count, self.sample_rate_hz)
        while len(samples) < self.sample_count and not rospy.is_shutdown():
            if self.latest_angles:
                samples.append(self.latest_angles[:])
                rospy.logdebug("Sample %d: %s", len(samples), self.latest_angles)
            rate.sleep()

        if not samples:
            return TriggerResponse(success=False, message="Failed to collect any samples.")

        # Compute element-wise average across collected samples
        averaged = []
        for i in range(len(samples[0])):
            total = 0.0
            for s in samples:
                total += s[i]
            averaged.append(total / len(samples))

        self.recorded_points.append(averaged)
        # Save all recorded points to the ROS Parameter Server
        rospy.set_param(self.param_key, self.recorded_points)

        idx = len(self.recorded_points)
        rospy.loginfo("Recorded point %d: %s", idx, averaged)

        # Format values to 6 decimal places
        values_str = ["%.6f" % v for v in averaged]
        response_msg = ("Reference point %d/%d recorded successfully. Values: %s" 
                       % (idx, self.max_points, values_str))
        if idx == self.max_points:
            response_msg += " Calibration complete."

        return TriggerResponse(success=True, message=response_msg)


def main():
    rospy.init_node('point_recorder', anonymous=False)
    recorder = PointRecorder()
    rospy.spin()

if __name__ == '__main__':
    main()

    # enter at ternimal -> rosservice call /record_point "{}"
