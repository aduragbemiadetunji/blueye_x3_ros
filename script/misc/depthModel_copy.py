#!/home/aduragbemi/.pyenv/shims/python
from EKF import ExtendedKalmanFilter

import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce

import tkinter as tk
from tkinter import ttk

input = np.array([0.0, 0.0, 0.0, 0.0]) 
dt = 0.01


state_publisher = rospy.Publisher("/blueye_x3/state/depthEKF_new", BlueyeState, queue_size=10)


class ObserverGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Blueye Drone Observer Tuner")

        # Pmc1 Slider pnc and mnc
        self.label_pnc1 = tk.Label(self.master, text="Pnc1")
        self.label_pnc1.grid(row=0, column=0, padx=10, pady=5)
        self.slider_pnc1 = ttk.Scale(self.master, from_=0, to=20, length=200, orient="horizontal", command=self.update_ref)
        self.slider_pnc1.grid(row=0, column=1, padx=10, pady=5)
        self.label_pnc1_value = tk.Label(self.master, text="0.2")
        self.label_pnc1_value.grid(row=0, column=2, padx=10, pady=5)

        # Pmc2 Slider pnc and mnc
        self.label_pnc2 = tk.Label(self.master, text="Pnc2")
        self.label_pnc2.grid(row=1, column=0, padx=10, pady=5)
        self.slider_pnc2 = ttk.Scale(self.master, from_=0, to=20, length=200, orient="horizontal", command=self.update_ref)
        self.slider_pnc2.grid(row=1, column=1, padx=10, pady=5)
        self.label_pnc2_value = tk.Label(self.master, text="0.2")
        self.label_pnc2_value.grid(row=1, column=2, padx=10, pady=5)

        # Pmc3 Slider pnc and mnc
        self.label_pnc3 = tk.Label(self.master, text="Pnc3")
        self.label_pnc3.grid(row=2, column=0, padx=10, pady=5)
        self.slider_pnc3 = ttk.Scale(self.master, from_=0, to=20, length=200, orient="horizontal", command=self.update_ref)
        self.slider_pnc3.grid(row=2, column=1, padx=10, pady=5)
        self.label_pnc3_value = tk.Label(self.master, text="0.2")
        self.label_pnc3_value.grid(row=2, column=2, padx=10, pady=5)

        # mnc1 Slider
        self.label_mnc1 = tk.Label(self.master, text="Mnc1")
        self.label_mnc1.grid(row=3, column=0, padx=10, pady=5)
        self.slider_mnc1 = ttk.Scale(self.master, from_=0, to=2, length=200, orient="horizontal", command=self.update_ref)
        self.slider_mnc1.grid(row=3, column=1, padx=10, pady=5)
        self.label_mnc1_value = tk.Label(self.master, text="0.01")
        self.label_mnc1_value.grid(row=3, column=2, padx=10, pady=5)

        # mnc2 Slider
        self.label_mnc2 = tk.Label(self.master, text="Mnc2")
        self.label_mnc2.grid(row=4, column=0, padx=10, pady=5)
        self.slider_mnc2 = ttk.Scale(self.master, from_=0, to=2, length=200, orient="horizontal", command=self.update_ref)
        self.slider_mnc2.grid(row=4, column=1, padx=10, pady=5)
        self.label_mnc2_value = tk.Label(self.master, text="0.01")
        self.label_mnc2_value.grid(row=4, column=2, padx=10, pady=5)

        # mnc3 Slider
        self.label_mnc3 = tk.Label(self.master, text="Mnc3")
        self.label_mnc3.grid(row=5, column=0, padx=10, pady=5)
        self.slider_mnc3 = ttk.Scale(self.master, from_=0, to=2, length=200, orient="horizontal", command=self.update_ref)
        self.slider_mnc3.grid(row=5, column=1, padx=10, pady=5)
        self.label_mnc3_value = tk.Label(self.master, text="0.01")
        self.label_mnc3_value.grid(row=5, column=2, padx=10, pady=5)

        self.tune_vals = [0.2, 0.2, 0.2, 0.01, 0.01, 0.0]  # Initial reference values
        # Initialize ROS node and subscriber
        rospy.init_node("blueye_EKF_depthModel_publisher")
        self.subscriber()

    def update_ref(self, event=None):
        # Update reference values when sliders are moved
        self.tune_vals[0] = round(self.slider_pnc1.get(), 2)
        self.tune_vals[1] = round(self.slider_pnc2.get(), 2)
        self.tune_vals[2] = round(self.slider_pnc3.get(), 2)

        self.tune_vals[3] = round(self.slider_mnc1.get(), 2)
        self.tune_vals[4] = round(self.slider_mnc2.get(), 2)
        self.tune_vals[5] = round(self.slider_mnc3.get(), 2)

        print(self.tune_vals)

        # Update labels with current values
        self.label_pnc1_value.config(text=str(self.tune_vals[0]))
        self.label_pnc2_value.config(text=str(self.tune_vals[1]))
        self.label_pnc3_value.config(text=str(self.tune_vals[2]))

        self.label_mnc1_value.config(text=str(self.tune_vals[3]))
        self.label_mnc2_value.config(text=str(self.tune_vals[4]))
        self.label_mnc3_value.config(text=str(self.tune_vals[5]))

    def subscriber(self):
        rospy.Subscriber("/blueye_x3/thrust_force", BlueyeForce, tauValues)
        rospy.Subscriber("/blueye_x3/state", BlueyeState, self.stateEstimator_new)

    def stateEstimator_new(self, msg):
        stateEstimator(msg, self.tune_vals)




def state_transition_function(state, input):
    # x, y, z, psi, u, v, w, r, b_x, b_y, b_z, b_psi = state
    tau1, tau2, tau3, tau4 = input[0], input[1], input[2], input[3] #15, 12, 5, 6  # Updated values
    # Wiener processes for bx, by, bz, and bpsi

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


def stateEstimator(msg, tune_vals):
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
    # process_noise_covariance = np.diag([0.6, 0.6, 0.6]) #zdot, wdot, bzdot
    # measurement_noise_covariance = np.diag([0.005, 0.005]) #z, w

    process_noise_covariance = np.diag([tune_vals[0], tune_vals[1], tune_vals[2]]) #zdot, wdot, bzdot
    measurement_noise_covariance = np.diag([tune_vals[3], tune_vals[4]]) #z, w

    # Create an instance of the ExtendedKalmanFilter class

    ekf = ExtendedKalmanFilter(initial_state, initial_covariance, process_noise_covariance, measurement_noise_covariance, state_transition_function, observation_function, state_transition_jacobian, observation_jacobian, error_jacobian, input)
    # ekf = ExtendedKalmanFilter()

    # Run the predict and update steps
    ekf.predict()
    # Provide the observed measurement for the update step
    # observed_measurement = np.array([1, 2, 3])  # Replace with actual observed measurement

    observed_measurement = np.array([state_z, state_w])
    ekf.update(observed_measurement)

    state_EKF_msg.z = ekf.state_estimate[0]
    state_EKF_msg.w = ekf.state_estimate[1]
    state_EKF_msg.bz = ekf.state_estimate[2]

    state_publisher.publish(state_EKF_msg)


# if __name__ == "__main__":
#     rospy.init_node("blueye_EKF_depthModel_publisher")
#     # state_publisher = rospy.Publisher("/blueye_x3/state/EKF", BlueyeState, queue_size=10)
#     state_publisher = rospy.Publisher("/blueye_x3/state/depthEKF", BlueyeState, queue_size=10)
#     subscriber()
    
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         pass


def main():
    root = tk.Tk()
    app = ObserverGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()