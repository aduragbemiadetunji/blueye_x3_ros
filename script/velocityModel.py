#!/home/aduragbemi/.pyenv/shims/python
from EKF import ExtendedKalmanFilter

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

    u, v, b_u, b_v = state
    b_u += np.random.normal(scale=np.sqrt(dt))
    wb_u = np.random.normal(scale=np.sqrt(dt))
    b_v += np.random.normal(scale=np.sqrt(dt))
    wb_v = np.random.normal(scale=np.sqrt(dt))
    return np.array([
        0.073047*tau1 -0.134463*u - 0.7881*v + 0.073047*b_u - 0.073047*b_v,
        0.0357755*tau2 - 0.1254*u - 0.11363*v + 0.035729*b_u + 0.035729*b_v,
        wb_u - 100 * b_u,
        wb_v - 100 * b_v,
    ])

def state_transition_jacobian(state):
    # u, v = state
    return np.array([
        [-0.134463, -0.7881, 0.073047, -0.073047],
        [-0.1254, - 0.11363, 0.035729, 0.035729],
        [0, 0, -100, 0],
        [0, 0, 0, -100]
    ])

def observation_function(state):
    return np.array([
        state[0], state[1]
    ])

def observation_jacobian(state):
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
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

    model = 3 #1 for yaw and yawrate, 2 for depth and w, 3 for all velocities.

    initial_state = np.zeros(4)
    state_EKF_msg = BlueyeState()

    #I NEED TO GET THE INITIAL STATE AND STORE IT HERE, THEN i PASS IT INTO THE EKF TO USE
    if(init_state==True):
        initial_state = np.array([msg.u, msg.v, msg.bx, msg.by])
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

    initial_covariance = np.zeros(4)     
    process_noise_covariance = np.diag([0.6, 0.6, 0.6, 0.6]) #udot, vdot, budot, bvdot
    measurement_noise_covariance = np.diag([0.01, 0.01]) #u, v



    # Create an instance of the ExtendedKalmanFilter class

    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input)
    # ekf = ExtendedKalmanFilter()

    # Run the predict and update steps
    ekf.predict()
    # Provide the observed measurement for the update step
    # observed_measurement = np.array([1, 2, 3])  # Replace with actual observed measurement

    observed_measurement = np.array([state_u, state_v])
    ekf.update(observed_measurement)

    state_EKF_msg.u = ekf.state_estimate[0]
    state_EKF_msg.v = ekf.state_estimate[1]
    state_EKF_msg.bx = ekf.state_estimate[2]
    state_EKF_msg.by = ekf.state_estimate[3]


    state_publisher.publish(state_EKF_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_EKF_velocityModel_publisher")
    # state_publisher = rospy.Publisher("/blueye_x3/state/EKF", BlueyeState, queue_size=10)
    state_publisher = rospy.Publisher("/blueye_x3/state/velocityEKF", BlueyeState, queue_size=10)
    subscriber()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass