#!/home/aduragbemi/.pyenv/shims/python
import time
import rospy
from blueye_x3_ros.msg import BlueyeDepth, BlueyeState  # Import the ROS IMU message type


import blueye.protocol as bp
from blueye.sdk import Drone

def callback_depth(msg_type, msg):
    # Create an ROS IMU message
    depth_msg = BlueyeDepth()
    # state_msg = BlueyeState()


    depth_msg.header.stamp = rospy.Time.now()
    depth_msg.header.frame_id = "depth_link"  # Replace with your frame ID

    # state_msg.header.stamp = rospy.Time.now()
    # state_msg.header.frame_id = "imu_link"  # Replace with your frame ID
    
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

    # Add a callback for the DepthTel message, storing the ID for later use
    # callback_id = my_drone.telemetry.add_msg_callback([bp.DepthTel], callback_depth)

    # Adjust the publishing frequency to 5 Hz
    my_drone.telemetry.set_msg_publish_frequency(bp.DepthTel, 5)


    # Callback is triggered by a separate thread while we sleep here
    # time.sleep(5)

    cb_calibrated = my_drone.telemetry.add_msg_callback([bp.DepthTel], callback_depth)


    # Remove the callback using the ID we stored when it was created (not really necessary here
    # since the my_drone object goes out of scope immediately afterwards)
    # my_drone.telemetry.remove_msg_callback(callback_id)



    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
