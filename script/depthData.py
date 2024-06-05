#!/usr/bin/env python3
import time
import rospy
from blueye_x3_ros.msg import BlueyeDepth, BlueyeState  # Import the ROS IMU message type


import blueye.protocol as bp
from blueye.sdk import Drone

def callback_depth(msg_type, msg):
    # Create an ROS IMU message
    depth_msg = BlueyeDepth()


    depth_msg.header.stamp = rospy.Time.now()
    depth_msg.header.frame_id = "depth_link"  # Replace with your frame ID
    
    depth_msg.depth = msg.depth.value
    # state_msg.z = msg.depth.value
    
    # Publish the ROS IMU message
    imu_publisher.publish(depth_msg)
    # state_publisher.publish(state_msg)

if __name__ == "__main__":
    rospy.init_node("blueye_depth_publisher")
    imu_publisher = rospy.Publisher("/blueye_x3/depth", BlueyeDepth, queue_size=10)

    # state_publisher = rospy.Publisher("/blueye_x3/state", BlueyeState, queue_size=10)



    my_drone = Drone()
    print('Depth Sensor active')

    # Adjust the publishing frequency to 5 Hz
    my_drone.telemetry.set_msg_publish_frequency(bp.DepthTel, 5)


    # Callback is triggered by a separate thread while we sleep here
    #Call the depth value from the protocol and use it in the call back function as a ros topic.
    cb_calibrated = my_drone.telemetry.add_msg_callback([bp.DepthTel], callback_depth)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
