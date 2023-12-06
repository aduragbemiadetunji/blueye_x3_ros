#!/home/aduragbemi/.pyenv/shims/python
import math
import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForceRef

from blueye.sdk import Drone

# ref_x = 1.0
# ref_y = -1.5
# ref_z = 0.4
# ref_yaw = 30

ref_val = np.array([0.0, 0.0, 0.0, 0.0])

myDrone = Drone()

control = BlueyeForceRef()

init_ref = True


class PID:
    def __init__(self, setpoint, Kp=0, Ki=0, Kd=0, sample_time=0.1):
        self.setpoint = setpoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sample_time = sample_time
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0

    def compute(self, feedback_value):
        current_time = time.time()
        elapsed_time = (current_time - self.last_time)*1000000
        error = self.setpoint - feedback_value
        self.integral += error * elapsed_time
        derivative = (error - self.last_error) / elapsed_time
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.last_time = current_time
        self.last_error = error
        return output

	#how to use

	# pid = PID(setpoint=10, Kp=1, Ki=0.1, Kd=0.5, sample_time=0.1)
	# feedback_value = 5
	# output = pid.compute(feedback_value)
	# print(output)

def computeThrust(msg):
    global ref_val
    global init_ref

    
    if init_ref:
        surge_ref, sway_ref, heave_ref, yaw_ref = msg.x, msg.y, msg.z, np.deg2rad(msg.psi)
        control.surge_ref = surge_ref
        control.sway_ref = sway_ref
        control.heave_ref = heave_ref
        control.yaw_ref = yaw_ref
        ref_val = np.array([surge_ref, sway_ref, heave_ref, yaw_ref])
        init_ref = False
    # reference_point = np.array([ref_x, ref_y, ref_z, np.deg2rad(ref_yaw)])
    reference_point = ref_val
    Kp = np.array([0.5, 0.3, 1.9, 0.4]) #TUNE x, y, z, yaw 1.8 0.4
    Ki = np.array([0.2, 0.1, 0.4, 0.09]) #TUNE 0.3 0.09
    Kd = np.array([0.0, 0.0, 0.5, 0.4]) #TUNE 0.5 0.4

    pos_PID = PID(setpoint=reference_point, Kp=Kp, Ki=Ki, Kd=Kd)
    feedback_value = np.array([msg.x, msg.y, msg.z, np.deg2rad(msg.psi)])
    output = pos_PID.compute(feedback_value)
    # print(output)
    writeThrustValues(surge=output[0], sway=output[1], heave=output[2], yaw=output[3])
    # writeThrustValues(surge=0, sway=0, heave=-0.3, yaw=output[3])
    # print(output)

def writeThrustValues(surge=0, sway=0, heave=0, yaw=0):
    # print(surge)
    # print(sway)
    # print(heave)
    # print(yaw)
    # print(" ")

    # UNCOMMENT WHEN YOU HAVE DRONE
    myDrone.motion.surge = surge
    myDrone.motion.sway = sway
    myDrone.motion.heave = heave
    myDrone.motion.yaw = yaw

    control.surge = surge
    control.sway = sway
    control.heave = heave
    control.yaw = yaw

    control_publisher.publish(control)
    #test if when the robot is not moving, the thrust_force is 0, then you move joystick it changes, then lastly, applying a surge value here, does it change the thrust_force


def subscriber():
    rospy.Subscriber("/blueye_x3/state/EKF_new", BlueyeState, computeThrust)




if __name__ == "__main__":
    rospy.init_node("blueye_controller_publisher")
    control_publisher = rospy.Publisher("/blueye_x3/tau", BlueyeForceRef, queue_size=10) #thrust_force

    subscriber()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass