#!/home/aduragbemi/.pyenv/shims/python

import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce, BlueyeImu


input = np.array([0.0, 0.0, 0.0, 0.0]) 
acc = np.array([0.0, 0.0, 0.0]) 


prev_vel = None
prev_time = None
bias = np.array([0.0, 0.0, 0.0, 0.0])



def subscriber():
    rospy.Subscriber("/blueye_x3/imu", BlueyeImu, computeAcc)
    rospy.Subscriber("/blueye_x3/thrust_force", BlueyeForce, tauValues)
    rospy.Subscriber("/blueye_x3/state", BlueyeState, computeBias)
    rospy.Subscriber("/blueye_x3/state/velocityEKF", BlueyeState, computeBiasError)


def computeAcc(msg):
    global acc
    acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    


def tauValues(msg):
    global input
    input = np.array([msg.surge, msg.sway, msg.heave, msg.yaw])
    


def computeBias(msg):
    global input
    global prev_time, prev_r_vel
    global bias
    global acc

    vel = np.array([msg.u, msg.v, msg.w, msg.r])
    r_vel = np.array([msg.r])

    current_time = rospy.get_time()

    Mass = np.diag([13.6899, 27.9882, 19.5895, 0.2480])
    Damping = np.diag([1.8430, 3.1802, 0.3778, 0.0868])

    # print(acc)


    if prev_vel is not None and prev_time is not None:
        dt = current_time - prev_time
        r_dot = (r_vel - prev_r_vel) / dt
        vel_dot = np.array([acc, r_dot])
        bias = ((Mass * vel_dot) + (Damping * vel) - input)[0]

        # rospy.loginfo(bias)

    prev_r_vel = r_vel
    prev_time = current_time

def computeBiasError(msg):
    global biass
    biasError_msg = BlueyeState()



    bias_est = np.array([msg.bx, msg.by, msg.bz, msg.bpsi])
    bias_error = bias_est - bias

    biasError_msg.bx = bias[0]
    biasError_msg.by = bias_est[0]
    biasError_msg.bz = bias_error[0]
    biasError_msg.bpsi = bias_error[0]

    biasError_publisher.publish(biasError_msg)


if __name__ == "__main__":
    rospy.init_node("bias_analysis_publisher")
    biasError_publisher = rospy.Publisher("/blueye_x3/biasError", BlueyeState, queue_size=10)
    subscriber()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass