import time
import rospy
from sensor_msgs.msg import BlueyeImu  # Import the ROS IMU message type

import blueye.protocol as bp
from blueye.sdk import Drone

def callback_imu_raw(msg_type, msg):
    # Create an ROS IMU message
    imu_msg = BlueyeImu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu_link"  # Replace with your frame ID
    
    # Fill in the IMU data from the raw data received from the drone
    # You'll need to adapt this based on the structure of the raw data
    imu_msg.linear_acceleration.x = msg.acceleration_x
    imu_msg.linear_acceleration.y = msg.acceleration_y
    imu_msg.linear_acceleration.z = msg.acceleration_z
    imu_msg.angular_velocity.x = msg.angular_velocity_x
    imu_msg.angular_velocity.y = msg.angular_velocity_y
    imu_msg.angular_velocity.z = msg.angular_velocity_z
    
    # Publish the ROS IMU message
    imu_publisher.publish(imu_msg)

if __name__ == "__main__":
    rospy.init_node("blueye_imu_publisher")
    imu_publisher = rospy.Publisher("/blueye_x3/imu", BlueyeImu, queue_size=10)

    my_drone = Drone()
    my_drone.telemetry.set_msg_publish_frequency(bp.Imu1Tel, 10)
    my_drone.telemetry.set_msg_publish_frequency(bp.Imu2Tel, 10)

    cb_raw = my_drone.telemetry.add_msg_callback([bp.Imu1Tel, bp.Imu2Tel], callback_imu_raw)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
