#!/home/aduragbemi/.pyenv/shims/python
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForceRef
from blueye.sdk import Drone
import numpy as np
from PID import PID

import tkinter as tk
import customtkinter as ctk
from tkinter import ttk

ctk.set_appearance_mode("system")
ctk.set_default_color_theme("dark-blue")



# myDrone = Drone()
control_publisher = rospy.Publisher("/blueye_x3/tau", BlueyeForceRef, queue_size=10)
control = BlueyeForceRef()


def writeThrustValues(surge=0, sway=0, heave=0, yaw=0, offset_vals = 0):
    # myDrone.motion.surge = surge
    # myDrone.motion.sway = sway
    # myDrone.motion.heave = heave
    # myDrone.motion.yaw = yaw

    control.surge = surge + offset_vals[0]
    control.sway = sway + offset_vals[1]
    control.heave = heave
    control.yaw = yaw

    control_publisher.publish(control)


def normalize_angle(angle):
    """ Normalize an angle to [-180, 180] interval """
    normalized_angle = np.deg2rad((angle + 180) % 360 - 180)
    return normalized_angle


def computeThrust(msg, ref_vals, offset_vals, control_sw):
    # print(ref_vals)
    Kp = np.array([1.3, 3.0, 1.8, 0.4]) #TUNE x, y, z, yaw 0.5 0.3 1.8 0.4 ---u,v 0.03, 0.0005
    Ki = np.array([0.0, 0.05, 0.3, 0.09]) #TUNE 0.2 0.1 0.3 0.09 --- 0.06, 0.002
    Kd = np.array([0.1, 1.0, 0.5, 0.4]) #TUNE 0 0 0.5 0.4 -----0.001, 0.001
    # rospy.loginfo(ref_vals) 

    if control_sw:
        pos_PID = PID(setpoint=[ref_vals[0],ref_vals[1],ref_vals[3],ref_vals[5]], Kp=Kp, Ki=Ki, Kd=Kd)
        feedback_value = np.array([msg.u, msg.v, msg.w, msg.r]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)
    else:
        pos_PID = PID(setpoint=[ref_vals[0],ref_vals[1],ref_vals[2],normalize_angle(ref_vals[4])], Kp=Kp, Ki=Ki, Kd=Kd)
        feedback_value = np.array([msg.u, msg.v, msg.z, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)
    output = pos_PID.compute(feedback_value)
    # print(output)
    writeThrustValues(surge=output[0], sway=output[1], heave=output[2], yaw=output[3], offset_vals=offset_vals)
    # writeThrustValues(surge=0, sway=0, heave=-0.3, yaw=output[3])

class BlueyeControllerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Blueye Drone Reference Controller")

        self.current_topic = "/blueye_x3/state/EKF_coupled"

        self.control_sw = True #True for velocity control and False for position control

        # Surge Slider
        self.label_surge = ctk.CTkLabel(self.master, text="Surge")
        self.label_surge.grid(row=0, column=0, padx=10, pady=5)
        self.slider_surge = ctk.CTkSlider(self.master, from_=-0.5, to=0.5, orientation="horizontal", command=self.update_ref)
        self.slider_surge.grid(row=0, column=1, padx=10, pady=5)
        self.label_surge_value = ctk.CTkLabel(self.master, text="0.0")
        self.label_surge_value.grid(row=0, column=2, padx=10, pady=5)

        # Sway Slider
        self.label_sway = ctk.CTkLabel(self.master, text="Sway")
        self.label_sway.grid(row=1, column=0, padx=10, pady=5)
        self.slider_sway = ctk.CTkSlider(self.master, from_=-0.5, to=0.5, orientation="horizontal", command=self.update_ref)
        self.slider_sway.grid(row=1, column=1, padx=10, pady=5)
        self.label_sway_value = ctk.CTkLabel(self.master, text="0.0")
        self.label_sway_value.grid(row=1, column=2, padx=10, pady=5)

        # Heave Slider
        self.label_heave = ctk.CTkLabel(self.master, text="Heave")
        self.label_heave.grid(row=2, column=0, padx=10, pady=5)
        self.slider_heave = ctk.CTkSlider(self.master, from_=-1, to=1, orientation="horizontal", command=self.update_ref)
        self.slider_heave.grid(row=2, column=1, padx=10, pady=5)
        self.label_heave_value = ctk.CTkLabel(self.master, text="0.0")
        self.label_heave_value.grid(row=2, column=2, padx=10, pady=5)

        # Heave rate Slider
        self.label_heave_rate = ctk.CTkLabel(self.master, text="Heave_Rate")
        self.label_heave_rate.grid(row=3, column=0, padx=10, pady=5)
        self.slider_heave_rate = ctk.CTkSlider(self.master, from_=-1, to=1, orientation="horizontal", command=self.update_ref)
        self.slider_heave_rate.grid(row=3, column=1, padx=10, pady=5)
        self.label_heave_rate_value = ctk.CTkLabel(self.master, text="0.0")
        self.label_heave_rate_value.grid(row=3, column=2, padx=10, pady=5)

        # Yaw Slider
        self.label_yaw = ctk.CTkLabel(self.master, text="Yaw")
        self.label_yaw.grid(row=4, column=0, padx=10, pady=5)
        self.slider_yaw = ctk.CTkSlider(self.master, from_=0, to=360, orientation="horizontal", command=self.update_ref)
        self.slider_yaw.grid(row=4, column=1, padx=10, pady=5)
        self.label_yaw_value = ctk.CTkLabel(self.master, text="0.0")
        self.label_yaw_value.grid(row=4, column=2, padx=10, pady=5)

        # Yaw rate Slider
        self.label_yaw_rate = ctk.CTkLabel(self.master, text="Yaw_Rate")
        self.label_yaw_rate.grid(row=5, column=0, padx=10, pady=5)
        self.slider_yaw_rate = ctk.CTkSlider(self.master, from_=-1, to=1, orientation="horizontal", command=self.update_ref)
        self.slider_yaw_rate.grid(row=5, column=1, padx=10, pady=5)
        self.label_yaw_rate_value = ctk.CTkLabel(self.master, text="0.0")
        self.label_yaw_rate_value.grid(row=5, column=2, padx=10, pady=5)

        # Surge Offset Slider
        self.surge_offset = ctk.CTkLabel(self.master, text="Surge Offset")
        self.surge_offset.grid(row=6, column=0, padx=10, pady=5)
        self.slider_surge_offset = ctk.CTkSlider(self.master, from_=-1/2, to=1/2, orientation="horizontal", command=self.update_ref)
        self.slider_surge_offset.grid(row=6, column=1, padx=10, pady=5)
        self.surge_offset_value = ctk.CTkLabel(self.master, text="0.0")
        self.surge_offset_value.grid(row=6, column=2, padx=10, pady=5)

        # Sway Offset Slider
        self.sway_offset = ctk.CTkLabel(self.master, text="Sway Offset")
        self.sway_offset.grid(row=7, column=0, padx=10, pady=5)
        self.slider_sway_offset = ctk.CTkSlider(self.master, from_=-1.5, to=1.5, orientation="horizontal", command=self.update_ref)
        self.slider_sway_offset.grid(row=7, column=1, padx=10, pady=5)
        self.sway_offset_value = ctk.CTkLabel(self.master, text="0.0")
        self.sway_offset_value.grid(row=7, column=2, padx=10, pady=5)

        self.model_selector = ctk.StringVar(value="on")
        self.model_switch = ctk.CTkSwitch(self.master, text="Model Selector", command=self.model_switch_event,
                                        variable=self.model_selector, onvalue="on", offvalue="off")
        
        self.model_switch.grid()

        self.control_selector = ctk.StringVar(value="on")
        self.control_switch = ctk.CTkSwitch(self.master, text="Control Selector", command=self.control_switch_event,
                                        variable=self.control_selector, onvalue="on", offvalue="off")
        
        self.control_switch.grid()

        

        self.ref_vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial reference values

        self.offset_values = [0.0, 0.0]

        # Initialize ROS node and subscriber
        rospy.init_node("blueye_controller_publisher")
        self.subscriber = rospy.Subscriber(self.current_topic, BlueyeState, self.computeThrust)

        # self.subscriber()

    def model_switch_event(self):

        if self.model_selector.get() == "on":
            # rospy.Subscriber("/blueye_x3/state/velocityEKF", BlueyeState, self.computeThrust).unregister()
            self.topic_name = "/blueye_x3/state/EKF_coupled" 
        else:
            # rospy.Subscriber("/blueye_x3/state/EKF_coupled", BlueyeState, self.computeThrust).unregister()
            self.topic_name = "/blueye_x3/state/EKF_decoupled"

        self.subscriber.unregister()
        self.subscriber = rospy.Subscriber(self.topic_name, BlueyeState, self.computeThrust)
        self.current_topic = self.topic_name
        # self.subscriber.unregister()
        # self.subscriber()

        

            # print(self.topic_name)

            # print("switch toggled, current value:", self.model_selector.get())

    def control_switch_event(self):
        # print(self.control_selector.get())
        if self.control_selector.get() == "on":
            self.control_sw = True
        else:
            self.control_sw = False

            

    def update_ref(self, event=None):
        # Update reference values when sliders are moved
        self.ref_vals[0] = round(self.slider_surge.get(), 2)
        self.ref_vals[1] = round(self.slider_sway.get(), 2)
        self.ref_vals[2] = round(self.slider_heave.get(), 2)
        self.ref_vals[3] = round(self.slider_heave_rate.get(), 2)
        self.ref_vals[4] = round(self.slider_yaw.get(), 2)
        self.ref_vals[5] = round(self.slider_yaw_rate.get(), 2)
        # self.ref_vals[3] = round(np.deg2rad(self.slider_yaw.get()), 2)

        self.offset_values[0] = round(self.slider_surge_offset.get(), 3)
        self.offset_values[1] = round(self.slider_sway_offset.get(), 3)

        control.surge_ref = self.ref_vals[0]
        control.sway_ref = self.ref_vals[1]
        control.heave_ref = self.ref_vals[2]
        control.yaw_ref = self.ref_vals[4]

        # print(self.offset_values)

        # Update labels with current values
        self.label_surge_value.configure(text=str(self.ref_vals[0]))
        self.label_sway_value.configure(text=str(self.ref_vals[1]))
        self.label_heave_value.configure(text=str(self.ref_vals[2]))
        self.label_heave_rate_value.configure(text=str(self.ref_vals[3]))
        self.label_yaw_value.configure(text=str(self.ref_vals[4]))
        self.label_yaw_rate_value.configure(text=str(self.ref_vals[5]))

        self.surge_offset_value.configure(text=str(self.offset_values[0]))
        self.sway_offset_value.configure(text=str(self.offset_values[1]))


    # def subscriber(self):
    #     # rospy.Subscriber("/blueye_x3/state/velocityEKF", BlueyeState, self.computeThrust)
    #     print(self.topic_name)
    #     rospy.Subscriber(self.topic_name, BlueyeState, self.computeThrust)

    def computeThrust(self, msg):
        computeThrust(msg, self.ref_vals, self.offset_values, self.control_sw)


def main():
    root = ctk.CTk()
    root.geometry("720x480")
    app = BlueyeControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()