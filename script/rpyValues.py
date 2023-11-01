#!/home/aduragbemi/.pyenv/shims/python

import rospy
from blueye_x3_ros.msg import BlueyeImu, BlueyeDepth, BlueyeState  # Import the ROS IMU message type
# from sensor_msgs.msg import Imu  # Import the ROS IMU message type

import math

import time

# Constants
dt = 0.1  # Time step in seconds

# Initializations
gyro_angle = 0  # Initial gyro angle
complementary_angle = 0  # Initial complementary filter angle


# # Complementary filter function
# def complementary_filter(gyro_data, accel_data, alpha):
#     gyro_angle = gyro_data * dt
#     accel_angle = math.atan2(accel_data[1], accel_data[2]) * 180 / math.pi
#     return alpha * (complementary_angle + gyro_angle) + (1 - alpha) * accel_angle

# Define the callback function to process the received message
def imuCallback(imu_msg):

    accelerationX = (float(imu_msg.linear_acceleration.x) * 3.9);
    accelerationY = (float(imu_msg.linear_acceleration.y) * 3.9);
    accelerationZ = (float(imu_msg.linear_acceleration.z) * 3.9);

    gyro_z = imu_msg.angular_velocity.z
    state_msg = BlueyeState()
    state_msg.r = gyro_z

    pitch = 180 * math.atan (accelerationX/math.sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/math.pi;
    roll = 180 * math.atan (accelerationY/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/math.pi;
    yaw = 180 * math.atan (accelerationZ/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/math.pi;

    state_msg.psi = yaw

    rpy = roll, pitch, yaw
    print(rpy)
    state_publisher.publish(state_msg)

    # print(f"Yaw Rate: {yaw_rate}")

    # rospy.loginfo(rpy)  # Print the received message to the console


def depthCallback(depth_msg):
    state_msg = BlueyeState()

    state_msg.z = depth_msg.depth

    state_publisher.publish(state_msg)



def blueyeImuListner():
    # Initialize the ROS node
    rospy.init_node('blueye_imu_subscriber', anonymous=True)

    # Subscribe to the specified topic
    rospy.Subscriber("/blueye_x3/imu", BlueyeImu, imuCallback)
    rospy.Subscriber("/blueye_x3/depth", BlueyeDepth, depthCallback)


    # Keep the script running and continue to listen for messages
    rospy.spin()

if __name__ == '__main__':
    state_publisher = rospy.Publisher("/blueye_x3/state", BlueyeState, queue_size=10)

    blueyeImuListner()

