#!/usr/bin/env python3
import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce

input = np.array([0.0, 0.0, 0.0, 0.0]) 
    
class ExtendedKalmanFilter:
    def __init__(self, initial_state, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input):

        #####for my project#####
        initial_covariance = np.zeros((12, 12))

        pnc = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
        mnc = [0.005, 0.005, 0.002, 0.01, 0.01, 0.01, 0.01, 0.01]
        process_noise_covariance = np.diag(pnc)
        measurement_noise_covariance = np.diag(mnc)
        #############################

        self.state_estimate = initial_state
        self.covariance_estimate = initial_covariance
        self.Q = process_noise_covariance
        self.R = measurement_noise_covariance
        self.state_transition_function = state_transition_function
        self.observation_function = observation_function
        self.state_transition_jacobian = state_transition_jacobian
        self.observation_jacobian = observation_jacobian
        self.error_jacobian = error_jacobian
        self.input = input



    def predict(self):
        # Predict step
        self.state_estimate = self.state_transition_function(self.state_estimate, self.input)
        F = self.state_transition_jacobian(self.state_estimate)
        self.covariance_estimate = F @ self.covariance_estimate @ F.T + self.Q #  self.error_jacobian @ self.Q  @ (self.error_jacobian).T)

    def update(self, observed_measurement):
        # Update step
        H = self.observation_jacobian(self.state_estimate)
        y = observed_measurement - self.observation_function(self.state_estimate)
        S = np.dot(np.dot(H, self.covariance_estimate), H.T) + self.R
        K = np.dot(np.dot(self.covariance_estimate, H.T), np.linalg.inv(S))
        self.state_estimate = self.state_estimate + np.dot(K, y)
        I = np.eye(self.covariance_estimate.shape[0])
        self.covariance_estimate = np.dot((I - np.dot(K, H)), self.covariance_estimate)


def state_transition_function(state, input):
    x, y, z, psi, u, v, w, r, b_x, b_y, b_z, b_psi = state

    tau1, tau2, tau3, tau4 = input[0], input[1], input[2], input[3] #15, 12, 5, 6  # Updated values
    # Wiener processes for bx, by, bz, and bpsi
    dt = 0.01
    b_x += np.random.normal(scale=np.sqrt(dt))
    b_y += np.random.normal(scale=np.sqrt(dt))
    b_z += np.random.normal(scale=np.sqrt(dt))
    b_psi += np.random.normal(scale=np.sqrt(dt))
    # Zero-mean Gaussian noise for wbx, wby, wbz, and wbpsi
    wb_x = np.random.normal(scale=np.sqrt(dt))
    wb_y = np.random.normal(scale=np.sqrt(dt))
    wb_z = np.random.normal(scale=np.sqrt(dt))
    wb_psi = np.random.normal(scale=np.sqrt(dt))
    # Given state transition function

    return np.array([
        np.cos(psi) * u - np.sin(psi) * v,
        np.cos(psi) * v + np.sin(psi) * u,
        w,
        r,
        0.073047 * tau1 - 0.13463 * u - 0.7881 * r * v + 0.073047 * np.cos(psi) * b_x - 0.073047 * np.sin(psi) * b_y,
        0.0357755 * tau2 - 0.113636 * v - 0.1254 * r * u + 0.035729 * np.cos(psi) * b_y + 0.035729 * np.sin(psi) * b_x,
        0.051048 * b_z + 0.051048 * tau3 + 0.019286 * w, # -0.019286
        4.0328 * b_psi + 0.35005 * r + 4.0328 * tau4 + 57.6623 * u * v, #-0.35005
        wb_x - 100 * b_x,
        wb_y - 100 * b_y,
        wb_z - 100 * b_z,
        wb_psi - 100 * b_psi
    ])


# Define the state transition Jacobian matrix
def state_transition_jacobian(state):
    psi = state[3]
    u, v, r, b_x, b_y = state[4], state[5], state[7], state[8], state[9]
    return np.array([
        [0, 0, 0, -1.0 * np.cos(psi) * v - 1.0 * np.sin(psi) * u, np.cos(psi), -1.0 * np.sin(psi), 0, 0, 0, 0, 0, 0],
        [0, 0, 0, np.cos(psi) * u - 1.0 * np.sin(psi) * v, np.sin(psi), np.cos(psi), 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0],
        [0, 0, 0, -0.07305 * np.cos(psi) * b_y - 0.07305 * np.sin(psi) * b_x, -0.1346, -0.788 * r, 0, -0.788 * v, 0.07305 * np.cos(psi), -0.07305 * np.sin(psi), 0, 0],
        [0, 0, 0, 0.03573 * np.cos(psi) * b_x - 0.03573 * np.sin(psi) * b_y, -0.1254 * r, -0.1136, 0, -0.1254 * u, 0.03573 * np.sin(psi), 0.03573 * np.cos(psi), 0, 0],
        [0, 0, 0, 0, 0, 0, 0.01929, 0, 0, 0, 0.05105, 0],
        [0, 0, 0, 0, 57.66 * v, 57.66 * u, 0, 0.35, 0, 0, 0, 4.033],
        [0, 0, 0, 0, 0, 0, 0, 0, -100.0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, -100.0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -100.0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -100.0]
    ])


def observation_function(state):
    x, y, z, psi, u, v, w, r = state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7]
    return np.array([x, y, z, psi, u, v, w, r])


def observation_jacobian(state):
    return np.array([
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]
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

    initial_state = np.zeros(12)
    state_EKF_msg = BlueyeState()

    #I NEED TO GET THE INITIAL STATE AND STORE IT HERE, THEN i PASS IT INTO THE EKF TO USE
    if(init_state):
        initial_state = np.array([msg.x, msg.y, msg.z, msg.psi, msg.u, msg.v, msg.w, msg.r, msg.bx, msg.by, msg.bz, msg.bpsi])
        init_state = False

        # print(initial_state_not)

    # print(input)

    state_x = msg.x
    state_y = msg.y
    state_z = msg.z
    state_yaw = msg.psi
    state_u = msg.u
    state_v = msg.v
    state_w = msg.w
    state_yaw_rate = msg.r


    # Create an instance of the ExtendedKalmanFilter class
    ekf = ExtendedKalmanFilter(initial_state, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input)

    # Run the predict and update steps
    ekf.predict()
    # Provide the observed measurement for the update step
    observed_measurement = np.array([state_x, state_y, state_z, state_yaw, state_u, state_v, state_w, state_yaw_rate])  # Replace with actual observed measurement
    ekf.update(observed_measurement)

    state_EKF_msg.x = ekf.state_estimate[0]
    state_EKF_msg.y = ekf.state_estimate[1]
    state_EKF_msg.z = ekf.state_estimate[2]
    state_EKF_msg.psi = ekf.state_estimate[3]
    state_EKF_msg.u = ekf.state_estimate[4]
    state_EKF_msg.v = ekf.state_estimate[5]
    state_EKF_msg.w = ekf.state_estimate[6]
    state_EKF_msg.r = ekf.state_estimate[7]
    state_EKF_msg.bx = ekf.state_estimate[8]
    state_EKF_msg.by = ekf.state_estimate[9]
    state_EKF_msg.bz = ekf.state_estimate[10]
    state_EKF_msg.bpsi = ekf.state_estimate[11]

    #Publishng all the 12 states from this EKF model.
    state_publisher.publish(state_EKF_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_EKF_state_publisher")
    state_publisher = rospy.Publisher("/blueye_x3/state/EKF_coupled", BlueyeState, queue_size=10)
    subscriber()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    


