#!/home/aduragbemi/.pyenv/shims/python
from EKF import ExtendedKalmanFilter
from ObserverTuner import ObserverGUI

import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce



input = np.array([0.0, 0.0, 0.0, 0.0]) 
dt = 0.01

def state_transition_function(state, input):
    # x, y, z, psi, u, v, w, r, b_x, b_y, b_z, b_psi = state
    tau1, tau2, tau3, tau4 = input[0], input[1], input[2], input[3] #15, 12, 5, 6  # Updated values
    # Wiener processes for bx, by, bz, and bpsi

    psi, r, b_psi = state
    b_psi += np.random.normal(scale=np.sqrt(dt))
    wb_psi = np.random.normal(scale=np.sqrt(dt))
    return np.array([
        r,
        0.35005*r + 4.0328*tau4 + 4.0328 * b_psi,
        wb_psi - 100 * b_psi
    ])

def state_transition_jacobian(state):
    return np.array([
        [0, 1, 0],
        [0, 0.35005, 4.0328],
        [0, 0, -100]
    ])

def observation_function(state):
    return np.array([
        state[0], state[1]
    ])

def observation_jacobian(state):
        return np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])


error_jacobian = np.vstack((np.zeros((8, 4)), np.eye(4)))



def subscriber():
    rospy.Subscriber("/blueye_x3/thrust_force", BlueyeForce, tauValues)
    rospy.Subscriber("/blueye_x3/state", BlueyeState, stateEstimator)


def tauValues(msg):
    global input

    input = np.array([msg.surge, msg.sway, msg.heave, msg.yaw])


def stateEstimator(msg):
    global input
    init_state = True

    initial_state = np.zeros(3)
    state_EKF_msg = BlueyeState()

    #I NEED TO GET THE INITIAL STATE AND STORE IT HERE, THEN i PASS IT INTO THE EKF TO USE
    if(init_state==True):
        initial_state = np.array([msg.psi, msg.r, msg.bpsi])
        init_state = False

    # print(input)

    state_x = msg.x
    state_y = msg.y
    state_z = msg.z
    state_yaw = msg.psi
    state_u = msg.u
    state_v = msg.v
    state_w = msg.w
    state_yaw_rate = msg.r

    initial_covariance = np.zeros(3)     
    process_noise_covariance = np.diag([0.6, 0.6, 0.6])
    measurement_noise_covariance = np.diag([0.01, 0.01]) #psi, r


    # Create an instance of the ExtendedKalmanFilter class

    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input)
    # ekf = ExtendedKalmanFilter()

    # Run the predict and update steps
    ekf.predict()
    # Provide the observed measurement for the update step
    # observed_measurement = np.array([1, 2, 3])  # Replace with actual observed measurement
    observed_measurement = np.array([state_yaw, state_yaw_rate])
    ekf.update(observed_measurement)

    state_EKF_msg.psi = ekf.state_estimate[0]
    state_EKF_msg.r = ekf.state_estimate[1]
    state_EKF_msg.bpsi = ekf.state_estimate[2]


    state_publisher.publish(state_EKF_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_EKF_yawModel_publisher")
    # state_publisher = rospy.Publisher("/blueye_x3/state/EKF", BlueyeState, queue_size=10)
    state_publisher = rospy.Publisher("/blueye_x3/state/yawEKF", BlueyeState, queue_size=10)
    subscriber()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass