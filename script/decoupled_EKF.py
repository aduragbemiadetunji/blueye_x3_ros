#!/home/aduragbemi/.pyenv/shims/python
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeImu, BlueyeDepth  # Import the ROS IMU message type

from waterlinked_a50_ros_driver.msg import DVL

import blueye.protocol as bp
from blueye.sdk import Drone
import math

state_msg = BlueyeState()


def subscriber():
    rospy.Subscriber("/blueye_x3/state/depthEKF", BlueyeState, depthDecoupled)
    rospy.Subscriber("/blueye_x3/state/yawEKF", BlueyeState, yawDecoupled)
    rospy.Subscriber("/blueye_x3/state/velocityEKF", BlueyeState, velocityDecoupled)



def depthDecoupled(msg):
    state_msg.header.stamp = rospy.Time.now()
    state_msg.header.frame_id = "imu_link"
    state_msg.z = msg.z
    state_msg.w = msg.w
    state_msg.bz = msg.bz

def yawDecoupled(msg):
    state_msg.psi = msg.psi
    state_msg.r = msg.r
    state_msg.bpsi = msg.bpsi


def velocityDecoupled(msg):
    state_msg.u = msg.u
    state_msg.v = msg.v
    state_msg.bx = msg.bx
    state_msg.by = msg.by

    state_publisher.publish(state_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_state_publisher")
    state_publisher = rospy.Publisher("/blueye_x3/state/EKF_decoupled", BlueyeState, queue_size=10)
    subscriber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass