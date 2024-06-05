#!/usr/bin/env python3
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeImu, BlueyeDepth  # Import the ROS IMU message type

from waterlinked_a50_ros_driver.msg import DVL

import blueye.protocol as bp
from blueye.sdk import Drone
import math


posX, posY = 0, 0  # Initial position

state_msg = BlueyeState()
last_time = time.time()




def depthCallback(msg):
    # state_msg = BlueyeState()
    state_msg.header.stamp = rospy.Time.now()
    state_msg.header.frame_id = "depth_link"
    state_msg.z = msg.depth

def dvlCallback(msg):
    global posY, posX, last_time
     # Update position
    dt = 0.01  # Time step (for example, 10ms)
    current_time = time.time()
    # print(current_time)
    elapsedTime = current_time - last_time
    # print(elapsedTime)

    #Estimating the position from velocity by integrating
    posX += msg.velocity.x * elapsedTime
    posY += msg.velocity.y * elapsedTime

    # print(msg.velocity.x)
    # print(msg.velocity.y)
    # print(msg.velocity.z)
    # print(" ")
    state_msg.x = posX
    state_msg.y = posY
    state_msg.u = msg.velocity.x
    state_msg.v = msg.velocity.y
    state_msg.w = msg.velocity.z
    last_time = current_time


def imuCallback(msg):
    #yaw rate from z angular velocity measurement
    state_msg.r = msg.angular_velocity.z

    roll = msg.orientation.x
    pitch = msg.orientation.y
    yaw =  msg.orientation.z


    state_msg.psi = yaw

    state_publisher.publish(state_msg)


def subscriber():
    #subscribe to the three topics to build the measured state vector
    rospy.Subscriber("/blueye_x3/depth", BlueyeDepth, depthCallback)
    rospy.Subscriber("/dvl/data", DVL, dvlCallback)
    rospy.Subscriber("/blueye_x3/imu", BlueyeImu, imuCallback)


if __name__ == "__main__":
    rospy.init_node("blueye_state_publisher")
    state_publisher = rospy.Publisher("/blueye_x3/state", BlueyeState, queue_size=10)
    subscriber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
