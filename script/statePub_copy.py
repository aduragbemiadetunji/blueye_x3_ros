#!/home/aduragbemi/.pyenv/shims/python
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeImu, BlueyeDepth  # Import the ROS IMU message type

from waterlinked_a50_ros_driver.msg import DVL


import blueye.protocol as bp
from blueye.sdk import Drone
import math

state_msg = BlueyeState()


    # Initialize variables
posX, posY = 0, 0  # Initial position
velX, velY = 0, 0  # Initial velocity



def depthCallback(msg):
    # state_msg = BlueyeState()
    state_msg.header.stamp = rospy.Time.now()
    state_msg.header.frame_id = "imu_link"
    state_msg.z = msg.depth


def dvlCallback(msg):
    global posY, posX

     # Update position
    dt = 0.01  # Time step (for example, 10ms)

    posX += msg.velocity.x * dt
    posY += msg.velocity.y * dt

    # print(msg.velocity.x)
    # print(msg.velocity.y)
    # print(msg.velocity.z)
    # print(" ")
    state_msg.x = posX
    state_msg.y = posY
    state_msg.u = msg.velocity.x
    state_msg.v = msg.velocity.y
    state_msg.w = msg.velocity.z



def imuCallback(msg):
    # global posY, posX, velX, velY
    # accelerationX = float(msg.linear_acceleration.x) * 3.9
    # accelerationY = float(msg.linear_acceleration.y) * 3.9
    # accelerationZ = float(msg.linear_acceleration.z) * 3.9


    ######ADDED LINES########
    # accX = msg.linear_acceleration.x
    # accY = msg.linear_acceleration.y
    # accZ = msg.linear_acceleration.z


    dt = 0.01  # Time step (for example, 10ms)

    # Example acceleration data (accX, accY)
    # Replace these with your actual accelerometer data
    # acc_data = [(0.5, 0.3), (0.6, 0.4), ...]  # Replace with real data

    # for accX, accY in acc_data:
        # Update velocity
    # velX += accX * dt
    # velY += accY * dt

    # Update position
    # posX += velX * dt
    # posY += velY * dt


    ###########################
    gyro_z = msg.angular_velocity.z
    state_msg.r = gyro_z

    roll = msg.orientation.x
    pitch = msg.orientation.y
    yaw =  msg.orientation.z

    # pitch = 180 * math.atan(accelerationX / math.sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ)) / math.pi
    # roll = 180 * math.atan(accelerationY / math.sqrt(accelerationX * accelerationX + accelerationZ * accelerationZ)) / math.pi
    # yaw = 180 * math.atan(accelerationZ / math.sqrt(accelerationX * accelerationX + accelerationZ * accelerationZ)) / math.pi

    # state_msg.x = posX
    # state_msg.y = posY
    state_msg.psi = yaw

    state_publisher.publish(state_msg)





def subscriber():
    rospy.Subscriber("/blueye_x3/depth", BlueyeDepth, depthCallback)
    rospy.Subscriber("/dvl/data", DVL, dvlCallback)
    rospy.Subscriber("/blueye_x3/imu", BlueyeImu, imuCallback)




if __name__ == "__main__":
    rospy.init_node("blueye_state_publisher")
    state_publisher = rospy.Publisher("/blueye_x3/state_new", BlueyeState, queue_size=10)
    subscriber()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
