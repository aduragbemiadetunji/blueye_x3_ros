#!/usr/bin/env python3
from EKF import ExtendedKalmanFilter

import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce

input = np.array([0.0, 0.0, 0.0, 0.0]) 
dt = 0.01


def state_transition_function(state, input):
    tau1, tau2, tau3, tau4 = input[0], input[1], input[2], input[3] 

    # Wiener processes for bz
    z, w, b_z = state
    b_z += np.random.normal(scale=np.sqrt(dt))
    wb_z = np.random.normal(scale=np.sqrt(dt))
    return np.array([
        w,
        0.019286*w + 0.051048*tau3 + 0.051048 * b_z,
        wb_z - 100 * b_z,
    ])

def state_transition_jacobian(state):
    return np.array([
        [0, 1, 0],
        [0, 0.019286, 0.051048],
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
        initial_state = np.array([msg.z, msg.w, msg.bz])
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
    measurement_noise_covariance = np.diag([0.005, 0.005]) #z, w

    # Create an instance of the ExtendedKalmanFilter class

    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input)
    # ekf = ExtendedKalmanFilter()

    # Run the predict and update steps
    ekf.predict()
    # Provide the observed measurement for the update step
    observed_measurement = np.array([state_z, state_w])
    ekf.update(observed_measurement)

    state_EKF_msg.z = ekf.state_estimate[0]
    state_EKF_msg.w = ekf.state_estimate[1]
    state_EKF_msg.bz = ekf.state_estimate[2]

    #Publishng the three states(z, w, bz) from this EKF model.
    state_publisher.publish(state_EKF_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_EKF_depthModel_publisher")
    state_publisher = rospy.Publisher("/blueye_x3/state/depthEKF", BlueyeState, queue_size=10)
    subscriber()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass