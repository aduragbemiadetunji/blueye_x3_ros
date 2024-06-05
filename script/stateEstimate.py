#!/usr/bin/env python3
import time
import rospy
from blueye_x3_ros.msg import BlueyeDepth, BlueyeState  # Import the ROS IMU message type


import blueye.protocol as bp
from blueye.sdk import Drone

def callback_position_estimate(msg_type, msg):
    # Create an ROS IMU message
    state_est_msg = BlueyeState()
    # state_msg = BlueyeState()
    # print(msg.position_estimate.northing)


    state_est_msg.header.stamp = rospy.Time.now()
    state_est_msg.header.frame_id = "state_est_link"  # Replace with your frame ID
    
    state_est_msg.x = msg.position_estimate.northing
    state_est_msg.y = msg.position_estimate.easting
    state_est_msg.psi = msg.position_estimate.heading
    state_est_msg.u = msg.position_estimate.surge_rate
    state_est_msg.v = msg.position_estimate.sway_rate
    state_est_msg.r = msg.position_estimate.yaw_rate
    
    # Publish the ROS IMU message
    state_estimate_publisher.publish(state_est_msg)
    # state_publisher.publish(state_msg)

if __name__ == "__main__":
    rospy.init_node("blueye_state_estimate_publisher")
    state_estimate_publisher = rospy.Publisher("/blueye_x3/state_estimate", BlueyeState, queue_size=10)


    my_drone = Drone()
    print('Internal State Estimate Active')

    # Adjust the publishing frequency to 5 Hz
    my_drone.telemetry.set_msg_publish_frequency(bp.PositionEstimateTel, 5)

    #Getting the state estimate from the internal observer of the Blueye.
    cb_calibrated = my_drone.telemetry.add_msg_callback([bp.PositionEstimateTel], callback_position_estimate)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
