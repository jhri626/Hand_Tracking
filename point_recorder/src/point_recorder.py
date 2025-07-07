#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from typing import List, Optional

class PointRecorder:
    """
    PointRecorder node:
    - Subscribes to '/model_out' topic to receive angle arrays.
    - Provides a Trigger service 'record_point' to record averaged reference points.
    - Records up to 5 reference points and stores them on the ROS Parameter Server under 'calibration/recorded_points'.
    """

    def __init__(self):
        # Hardcoded configurations
        self.topic_name: str = '/model_out'
        self.service_name: str = 'record_point'
        self.sample_count: int = 20
        self.sample_rate_hz: int = 20
        self.max_points: int = 5
        self.param_key: str = 'calibration/recorded_points'

        # Internal state
        self.latest_angles: Optional[List[float]] = None
        self.recorded_points: List[List[float]] = []

        # Subscriber for angle data
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

    def angle_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback for '/model_out' topic.
        Stores the latest incoming angle array.
        """
        if not msg.data:
            rospy.logwarn("Received empty angle array on '%s'.", self.topic_name)
            return

        # Copy data to avoid aliasing issues
        self.latest_angles = list(msg.data)
        rospy.logdebug("Updated latest_angles: %s", self.latest_angles)

    def handle_record_point(self, req: Trigger.Request = None) -> TriggerResponse:
        """
        Service callback to record an averaged angle array as a new reference point.
        """
        if self.latest_angles is None:
            return TriggerResponse(success=False, message="No angle data received yet.")

        if len(self.recorded_points) >= self.max_points:
            msg = f"Maximum of {self.max_points} reference points reached."
            return TriggerResponse(success=False, message=msg)

        samples: List[List[float]] = []
        rate = rospy.Rate(self.sample_rate_hz)

        rospy.loginfo("Collecting %d samples at %d Hz.", self.sample_count, self.sample_rate_hz)
        while len(samples) < self.sample_count and not rospy.is_shutdown():
            if self.latest_angles:
                samples.append(self.latest_angles.copy())
                rospy.logdebug("Sample %d: %s", len(samples), self.latest_angles)
            rate.sleep()

        if not samples:
            return TriggerResponse(success=False, message="Failed to collect any samples.")

        # Compute element-wise average across collected samples
        averaged = [sum(values) / len(values) for values in zip(*samples)]

        self.recorded_points.append(averaged)
        # Save all recorded points to the ROS Parameter Server
        rospy.set_param(self.param_key, self.recorded_points)

        idx = len(self.recorded_points)
        rospy.loginfo("Recorded point %d: %s", idx, averaged)

        response_msg = (
            f"Reference point {idx}/{self.max_points} recorded successfully."
            f" Values: {[round(v, 6) for v in averaged]}"
        )
        if idx == self.max_points:
            response_msg += " Calibration complete."

        return TriggerResponse(success=True, message=response_msg)


def main():
    """
    Initialize ROS node and start the PointRecorder.
    """
    rospy.init_node('point_recorder', anonymous=False)
    recorder = PointRecorder()
    rospy.spin()

if __name__ == '__main__':
    main()
