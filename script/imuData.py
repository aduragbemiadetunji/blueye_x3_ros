#!/home/aduragbemi/.pyenv/shims/python
import time
import rospy
import math
# from sensor_msgs.msg import MagneticField
# from sensor_msgs.msg import Imu  # Import the ROS IMU message type
from blueye_x3_ros.msg import BlueyeImu  # Import the ROS IMU message type


import blueye.protocol as bp
from blueye.sdk import Drone

def callback_imu_calibrated(msg_type, msg):
    # Create an ROS IMU message
    imu_msg = BlueyeImu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu_link"  # Replace with your frame ID

    # magnet_msg = MagneticField()
    # magnet_msg.header.stamp = rospy.Time.now()
    # magnet_msg.header.frame_id = "magnetic_link"  # Replace with your frame ID


    # print(type(msg.imu.accelerometer.x))
    # Fill in the IMU data from the raw data received from the drone  #!/usr/bin/env python
    # You'll need to adapt this based on the structure of the raw data

    imu_msg.orientation.x = my_drone.pose['roll']
    imu_msg.orientation.y = my_drone.pose['pitch']
    imu_msg.orientation.z = my_drone.pose['yaw']

    imu_msg.linear_acceleration.x = msg.imu.accelerometer.x
    imu_msg.linear_acceleration.y = msg.imu.accelerometer.y
    imu_msg.linear_acceleration.z = msg.imu.accelerometer.z
    
    imu_msg.angular_velocity.x = msg.imu.gyroscope.x
    imu_msg.angular_velocity.y = msg.imu.gyroscope.y
    imu_msg.angular_velocity.z = msg.imu.gyroscope.z

    imu_msg.magnetic_field.x = msg.imu.magnetometer.x   
    imu_msg.magnetic_field.y = msg.imu.magnetometer.y
    imu_msg.magnetic_field.z = msg.imu.magnetometer.z
    
    # Publish the ROS IMU message
    imu_publisher.publish(imu_msg)
    # magnetic_publisher.publish(magnet_msg)


    #################Just to calculate rpy

    # accelerationX = (float(imu_msg.linear_acceleration.x) * 3.9);
    # accelerationY = (float(imu_msg.linear_acceleration.y) * 3.9);
    # accelerationZ = (float(imu_msg.linear_acceleration.z) * 3.9);

    # gyro_z = imu_msg.angular_velocity.z
    # state_msg = BlueyeState()
    # state_msg.r = gyro_z

    # pitch = 180 * math.atan (accelerationX/math.sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/math.pi;
    # roll = 180 * math.atan (accelerationY/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/math.pi;
    # yaw = 180 * math.atan (accelerationZ/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/math.pi;

    # state_msg.psi = yaw

    # rpy = roll, pitch, yaw
    # print(rpy)
    # state_publisher.publish(state_msg)
    # print(my_drone.pose['roll'])



    ####################################


if __name__ == "__main__":
    rospy.init_node("blueye_imu_publisher")
    imu_publisher = rospy.Publisher("/blueye_x3/imu", BlueyeImu, queue_size=10)

    # rospy.init_node("blueye_magnetic_publisher")
    # magnetic_publisher = rospy.Publisher("/blueye_x3/magnetic", MagneticField, queue_size=10)

    my_drone = Drone()
    print('Connected to Drone')
    my_drone.telemetry.set_msg_publish_frequency(bp.Imu1Tel, 10)
    my_drone.telemetry.set_msg_publish_frequency(bp.Imu2Tel, 10)
    my_drone.telemetry.set_msg_publish_frequency(bp.CalibratedImuTel, 10)

    # cb_raw = my_drone.telemetry.add_msg_callback([bp.Imu1Tel, bp.Imu2Tel], callback_imu_raw)
    cb_calibrated = my_drone.telemetry.add_msg_callback([bp.CalibratedImuTel], callback_imu_calibrated)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
