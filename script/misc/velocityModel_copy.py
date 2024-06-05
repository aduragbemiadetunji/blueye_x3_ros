#!/home/aduragbemi/.pyenv/shims/python
from EKF import ExtendedKalmanFilter

import numpy as np
import time
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForce

import tkinter as tk
from tkinter import ttk
import customtkinter as ctk

ctk.set_appearance_mode("system")
ctk.set_default_color_theme("dark-blue")

input = np.array([0.0, 0.0, 0.0, 0.0]) 
dt = 0.01

state_publisher = rospy.Publisher("/blueye_x3/state/velocityEKF_tune", BlueyeState, queue_size=10)


class ObserverGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Blueye Drone Velocity Observer Tuner")

        # Pmc1 Slider pnc and mnc
        self.label_pnc1 = ctk.CTkLabel(self.master, text="pnc1")#Udot
        self.label_pnc1.grid(row=0, column=0, padx=10, pady=5)
        self.slider_pnc1 = ctk.CTkSlider(self.master, from_=0, to=100, orientation="horizontal", command=self.update_ref)
        self.slider_pnc1.grid(row=0, column=1, padx=10, pady=5)
        self.label_pnc1_value = ctk.CTkLabel(self.master, text="0.6")
        self.label_pnc1_value.grid(row=0, column=2, padx=10, pady=5)

        # Pmc2 Slider pnc and mnc
        self.label_pnc2 = ctk.CTkLabel(self.master, text="pnc2") #Vdot
        self.label_pnc2.grid(row=1, column=0, padx=10, pady=5)
        self.slider_pnc2 = ctk.CTkSlider(self.master, from_=0, to=20, orientation="horizontal", command=self.update_ref)
        self.slider_pnc2.grid(row=1, column=1, padx=10, pady=5)
        self.label_pnc2_value = ctk.CTkLabel(self.master, text="0.6")
        self.label_pnc2_value.grid(row=1, column=2, padx=10, pady=5)

        # Pmc3 Slider pnc and mnc
        self.label_pnc3 = ctk.CTkLabel(self.master, text="pnc3")#B_udot
        self.label_pnc3.grid(row=2, column=0, padx=10, pady=5)
        self.slider_pnc3 = ctk.CTkSlider(self.master, from_=-2, to=50, orientation="horizontal", command=self.update_ref)
        self.slider_pnc3.grid(row=2, column=1, padx=10, pady=5)
        self.label_pnc3_value = ctk.CTkLabel(self.master, text="0.6")
        self.label_pnc3_value.grid(row=2, column=2, padx=10, pady=5)

        # mnc1 Slider
        self.label_mnc1 = ctk.CTkLabel(self.master, text="pnc4")#B_vdot
        self.label_mnc1.grid(row=3, column=0, padx=10, pady=5)
        self.slider_mnc1 = ctk.CTkSlider(self.master, from_=-2, to=20, orientation="horizontal", command=self.update_ref)
        self.slider_mnc1.grid(row=3, column=1, padx=10, pady=5)
        self.label_mnc1_value = ctk.CTkLabel(self.master, text="0.6")
        self.label_mnc1_value.grid(row=3, column=2, padx=10, pady=5)

        # mnc2 Slider
        self.label_mnc2 = ctk.CTkLabel(self.master, text="mnc1") #U
        self.label_mnc2.grid(row=4, column=0, padx=10, pady=5)
        self.slider_mnc2 = ctk.CTkSlider(self.master, from_=0, to=0.1, orientation="horizontal", command=self.update_ref)
        self.slider_mnc2.grid(row=4, column=1, padx=10, pady=5)
        self.label_mnc2_value = ctk.CTkLabel(self.master, text="0.01")
        self.label_mnc2_value.grid(row=4, column=2, padx=10, pady=5)

        # mnc3 Slider
        self.label_mnc3 = ctk.CTkLabel(self.master, text="mnc2")#V
        self.label_mnc3.grid(row=5, column=0, padx=10, pady=5)
        self.slider_mnc3 = ctk.CTkSlider(self.master, from_=0, to=2, orientation="horizontal", command=self.update_ref)
        self.slider_mnc3.grid(row=5, column=1, padx=10, pady=5)
        self.label_mnc3_value = ctk.CTkLabel(self.master, text="0.01")
        self.label_mnc3_value.grid(row=5, column=2, padx=10, pady=5)

        self.tune_vals = [0.6, 0.6, 0.6, 0.6, 0.01, 0.01]  # Initial reference values
        # Initialize ROS node and subscriber
        rospy.init_node("blueye_EKF_velocityModel_publisher")
        self.subscriber()

    def update_ref(self, event=None):
        # Update reference values when sliders are moved
        self.tune_vals[0] = round(self.slider_pnc1.get(), 2)
        self.tune_vals[1] = round(self.slider_pnc2.get(), 2)
        self.tune_vals[2] = round(self.slider_pnc3.get(), 2)

        self.tune_vals[3] = round(self.slider_mnc1.get(), 2)
        self.tune_vals[4] = round(self.slider_mnc2.get(), 5)
        self.tune_vals[5] = round(self.slider_mnc3.get(), 2)

        # print(self.tune_vals)

        # Update labels with current values
        self.label_pnc1_value.configure(text=str(self.tune_vals[0]))
        self.label_pnc2_value.configure(text=str(self.tune_vals[1]))
        self.label_pnc3_value.configure(text=str(self.tune_vals[2]))

        self.label_mnc1_value.configure(text=str(self.tune_vals[3]))
        self.label_mnc2_value.configure(text=str(self.tune_vals[4]))
        self.label_mnc3_value.configure(text=str(self.tune_vals[5]))

    def subscriber(self):
        rospy.Subscriber("/blueye_x3/thrust_force", BlueyeForce, tauValues)
        rospy.Subscriber("/blueye_x3/state", BlueyeState, self.stateEstimator_new)

    def stateEstimator_new(self, msg):
        stateEstimator(msg, self.tune_vals)


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


def stateEstimator(msg, tune_vals):
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
    # process_noise_covariance = np.diag([0.6, 0.6, 0.6, 0.6]) #udot, vdot, budot, bvdot
    # measurement_noise_covariance = np.diag([0.01, 0.01]) #u, v

    process_noise_covariance = np.diag([tune_vals[0], tune_vals[1], tune_vals[2], tune_vals[3]]) #udot, vdot, budot, bvdot
    measurement_noise_covariance = np.diag([tune_vals[4], tune_vals[5]]) #u, v



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


# if __name__ == "__main__":
#     rospy.init_node("blueye_EKF_velocityModel_publisher")
#     # state_publisher = rospy.Publisher("/blueye_x3/state/EKF", BlueyeState, queue_size=10)
#     state_publisher = rospy.Publisher("/blueye_x3/state/velocityEKF", BlueyeState, queue_size=10)
#     subscriber()
    
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         pass

def main():
    # root = tk.Tk()
    root = ctk.CTk()
    # root.geometry("500x480")
    app = ObserverGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()