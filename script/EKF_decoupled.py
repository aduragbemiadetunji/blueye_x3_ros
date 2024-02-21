#!/home/aduragbemi/.pyenv/shims/python

import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce

input = np.array([0.0, 0.0, 0.0, 0.0]) 

        
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


class DecoupledObserver:
    def __init__(self, model):
        self.model = model

    def initial_state(self):
        if self.model == 1:
            return np.zeros((3))
        elif self.model == 2:
            return np.zeros((3))
        elif self.model == 3:
            return np.zeros((4))
        
    def initial_covariance(self):
        if self.model == 1:
            return np.zeros(3)
        elif self.model == 2:
            return np.zeros(3)
        elif self.model == 3:
            return np.zeros(4)
        
    def process_noise_covariance(self):
        if self.model == 1:
            return np.diag([0.6, 0.6, 0.6])
        elif self.model == 2:
            return np.diag([0.6, 0.6, 0.6])
        elif self.model == 3:
            return np.diag([0.6, 0.6, 0.6, 0.6])

    def measurement_noise_covariance(self):
        if self.model == 1:
            return np.diag([0.01, 0.01]) #psi, r
        elif self.model == 2:
            return np.diag([0.005, 0.005]) #z, w
        elif self.model == 3:
            return np.diag([0.01, 0.01]) #u, v

    def state_transition_function(self, state, input):
        # x, y, z, psi, u, v, w, r, b_x, b_y, b_z, b_psi = state
        tau1, tau2, tau3, tau4 = input[0], input[1], input[2], input[3] #15, 12, 5, 6  # Updated values
        # Wiener processes for bx, by, bz, and bpsi
        dt = 0.01

        if self.model == 1:
            psi, r, b_psi = state
            b_psi += np.random.normal(scale=np.sqrt(dt))
            wb_psi = np.random.normal(scale=np.sqrt(dt))
            return np.array([
                r,
                0.35005*r + 4.0328*tau4 + 4.0328 * b_psi,
                wb_psi - 100 * b_psi
            ])
        elif self.model == 2:
            z, w, b_z = state
            b_z += np.random.normal(scale=np.sqrt(dt))
            wb_z = np.random.normal(scale=np.sqrt(dt))
            return np.array([
                w,
                0.019286*w + 0.051048*tau3 + 0.051048 * b_z,
                wb_z - 100 * b_z,
            ])
        elif self.model == 3:
            u, v, b_u, b_v = state
            b_u += np.random.normal(scale=np.sqrt(dt))
            wb_u = np.random.normal(scale=np.sqrt(dt))
            b_v += np.random.normal(scale=np.sqrt(dt))
            wb_v = np.random.normal(scale=np.sqrt(dt))
            return np.array([
                0.073047*tau1 -0.134463*u + 0.073047*b_u - 0.073047*b_v,
                0.0357755*tau2 - 0.11363*v + 0.035729*b_u + 0.035729*b_v,
                wb_u - 100 * b_u,
                wb_v - 100 * b_v,
            ])
    def state_transition_jacobian(self, state):
        if self.model == 1:
            return np.array([
                [0, 1, 0],
                [0, 0.35005, 4.0328],
                [0, 0, -100]
            ])
        elif self.model == 2:
            return np.array([
                [0, 1, 0],
                [0, 0.019286, 0.051048],
                [0, 0, -100]
            ])
        elif self.model == 3:
            # u, v = state
            return np.array([
                [-0.134463, 0, 0.073047, -0.073047],
                [0, - 0.11363, 0.035729, 0.035729],
                [0, 0, -100, 0],
                [0, 0, 0, -100]
            ])
    def observation_function(self, state):
        if self.model == 1:
            return np.array([
                state[0], state[1]
            ])
        elif self.model == 2:
            return np.array([
                state[0], state[1]
            ])
        elif self.model == 3:
            return np.array([
                state[0], state[1]
            ])
    def observation_jacobian(self, state):
        if self.model == 1:
            return np.array([
                [1, 0, 0],
                [0, 1, 0]
            ])
        elif self.model == 2:
            return np.array([
                [1, 0, 0],
                [0, 1, 0]
            ])
        elif self.model == 3:
            return np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0]
            ])


error_jacobian = np.vstack((np.zeros((8, 4)), np.eye(4)))

# input = np.array([0.5, 0.4, 0.6, 0.4]) #replace this with the values of the input going into the thrusters in body frame


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

    # initial_state = np.zeros(2)
    state_EKF_msg = BlueyeState()

    #I NEED TO GET THE INITIAL STATE AND STORE IT HERE, THEN i PASS IT INTO THE EKF TO USE
    if(init_state==True):
        # initial_state = np.array([msg.x, msg.y, msg.z, msg.psi, msg.u, msg.v, msg.w, msg.r, msg.bx, msg.by, msg.bz, msg.bpsi])
        if model == 1:
            # initial_state = np.zeros(2)
            initial_state = np.array([msg.psi, msg.r, msg.bpsi])
        elif(model == 2):
            initial_state = np.array([msg.z, msg.w, msg.bz])
        elif(model == 3):
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


    modelSelector = DecoupledObserver(model)
    initial_covariance = modelSelector.initial_covariance()
    process_noise_covariance = modelSelector.process_noise_covariance()
    measurement_noise_covariance = modelSelector.measurement_noise_covariance()
    state_transition_function = modelSelector.state_transition_function
    observation_function = modelSelector.observation_function
    state_transition_jacobian = modelSelector.state_transition_jacobian
    observation_jacobian = modelSelector.observation_jacobian



    # Create an instance of the ExtendedKalmanFilter class

    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input)
    # ekf = ExtendedKalmanFilter()

    # Run the predict and update steps
    ekf.predict()
    # Provide the observed measurement for the update step
    # observed_measurement = np.array([1, 2, 3])  # Replace with actual observed measurement
    if model == 1:
        observed_measurement = np.array([state_yaw, state_yaw_rate])
        ekf.update(observed_measurement)

        state_EKF_msg.psi = ekf.state_estimate[0]
        state_EKF_msg.r = ekf.state_estimate[1]
        state_EKF_msg.bpsi = ekf.state_estimate[2]

    elif model == 2:
        observed_measurement = np.array([state_z, state_w])
        ekf.update(observed_measurement)

        state_EKF_msg.z = ekf.state_estimate[0]
        state_EKF_msg.w = ekf.state_estimate[1]
        state_EKF_msg.bz = ekf.state_estimate[2]

    elif model == 3:
        observed_measurement = np.array([state_u, state_v])
        ekf.update(observed_measurement)

        state_EKF_msg.u = ekf.state_estimate[0]
        state_EKF_msg.v = ekf.state_estimate[1]
        state_EKF_msg.bx = ekf.state_estimate[2]
        state_EKF_msg.by = ekf.state_estimate[3]


    state_publisher.publish(state_EKF_msg)


if __name__ == "__main__":
    rospy.init_node("blueye_EKF_decoupledState_publisher")
    # state_publisher = rospy.Publisher("/blueye_x3/state/EKF", BlueyeState, queue_size=10)
    state_publisher = rospy.Publisher("/blueye_x3/state/EKF_decoupled", BlueyeState, queue_size=10)
    subscriber()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    