import numpy as np
# import time
# import rospy
# from blueye_x3_ros.msg import BlueyeState, BlueyeForce

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance,  state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input):

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
