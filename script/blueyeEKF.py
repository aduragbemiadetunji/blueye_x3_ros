#!/home/aduragbemi/.pyenv/shims/python
import time
import rospy
from blueye_x3_ros.msg import BlueyeState  # Import the ROS IMU message type


import blueye.protocol as bp
from blueye.sdk import Drone

def callback_estimate(msg_type, msg):
    print(bp.PositionEstimate().heading)

    # Create an ROS IMU message
    ekf_msg = BlueyeState()
    # state_msg = BlueyeState()
    print(msg)


    ekf_msg.header.stamp = rospy.Time.now()
    ekf_msg.header.frame_id = "ekf_blueye_link"  # Replace with your frame ID

    # state_msg.header.stamp = rospy.Time.now()
    # state_msg.header.frame_id = "imu_link"  # Replace with your frame ID
    
    # ekf_msg.x = msg.position_estimate.northing
    # state_msg.z = msg.depth.value
    # print(ekf_msg.x)
    
    # Publish the ROS IMU message
    ekf_publisher.publish(ekf_msg)
    # state_publisher.publish(state_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_state_EKF_publisher")
    ekf_publisher = rospy.Publisher("/blueye_x3/blueyeEKF", BlueyeState, queue_size=10)

    # state_publisher = rospy.Publisher("/blueye_x3/state", BlueyeState, queue_size=10)



    my_drone = Drone()
    print('Connected to Drone')

    # Add a callback for the DepthTel message, storing the ID for later use
    # callback_id = my_drone.telemetry.add_msg_callback([bp.DepthTel], callback_depth)

    # Adjust the publishing frequency to 5 Hz
    # my_drone.telemetry.set_msg_publish_frequency(bp.PositionEstimate, 5)


    # Callback is triggered by a separate thread while we sleep here
    # time.sleep(5)
    bp.ActivateGuestPortsCtrl()

    cb_calibrated = my_drone.telemetry.add_msg_callback([bp.PositionEstimate], callback_estimate)
    # print(dir(bp.GuestPortDeviceList()))
    # print(dir(bp.PositionEstimate()))



    # Remove the callback using the ID we stored when it was created (not really necessary here
    # since the my_drone object goes out of scope immediately afterwards)
    # my_drone.telemetry.remove_msg_callback(callback_id)



    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
