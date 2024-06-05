#!/usr/bin/env python3

import rospy

import blueye.protocol as bp
from blueye.sdk import Drone

from blueye_x3_ros.msg import BlueyeForce  # Import the ROS IMU message type


def callback_control_force(msg_type, msg): #msg_type
        # Create an ROS IMU message
    force_msg = BlueyeForce()
    force_msg.header.stamp = rospy.Time.now()
    force_msg.header.frame_id = "control_force_link"  # Replace with your frame ID

    force_msg.surge = msg.control_force.surge
    force_msg.sway = msg.control_force.sway
    force_msg.heave = msg.control_force.heave
    force_msg.yaw = msg.control_force.yaw
    # print(force_msg.surge)
    control_force_publisher.publish(force_msg)




if __name__ == "__main__":
    rospy.init_node("blueye_control_force_publisher")
    control_force_publisher = rospy.Publisher("/blueye_x3/thrust_force", BlueyeForce, queue_size=10)

    my_drone = Drone()
    print('Thrust Data active')
    my_drone.telemetry.set_msg_publish_frequency(bp.ControlForceTel, 10)

    #Get the estimated control force from the protocol
    cb_calibrated = my_drone.telemetry.add_msg_callback([bp.ControlForceTel], callback_control_force)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass