#!/home/aduragbemi/.pyenv/shims/python
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForceRef
from blueye.sdk import Drone
import numpy as np
from PID import PID

import tkinter as tk
from tkinter import ttk

myDrone = Drone()
control_publisher = rospy.Publisher("/blueye_x3/tau", BlueyeForceRef, queue_size=10)
control = BlueyeForceRef()


def writeThrustValues(surge=0, sway=0, heave=0, yaw=0, offset_vals = 0):
    myDrone.motion.surge = surge
    myDrone.motion.sway = sway
    myDrone.motion.heave = heave
    myDrone.motion.yaw = yaw

    control.surge = surge + offset_vals[0]
    control.sway = sway + offset_vals[1]
    control.heave = heave
    control.yaw = yaw

    control_publisher.publish(control)


def computeThrust(msg, ref_vals, offset_vals):
    # print(ref_vals)
    Kp = np.array([1.3, 2.5, 0, 0]) #TUNE x, y, z, yaw 0.5 0.3 1.8 0.4 ---u,v 0.03, 0.0005
    Ki = np.array([0.0, 0.0, 0, 0]) #TUNE 0.2 0.1 0.3 0.09 --- 0.06, 0.002
    Kd = np.array([0.1, 1.5, 0, 0]) #TUNE 0 0 0.5 0.4 -----0.001, 0.001
    # rospy.loginfo(ref_vals)
    pos_PID = PID(setpoint=ref_vals, Kp=Kp, Ki=Ki, Kd=Kd)
    feedback_value = np.array([msg.u, msg.v, msg.z, np.deg2rad(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)
    output = pos_PID.compute(feedback_value)
    # print(output)
    writeThrustValues(surge=output[0], sway=output[1], heave=output[2], yaw=output[3], offset_vals=offset_vals)
    # writeThrustValues(surge=0, sway=0, heave=-0.3, yaw=output[3])

class BlueyeControllerGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Blueye Drone Reference Controller")

        # Surge Slider
        self.label_surge = tk.Label(self.master, text="Surge")
        self.label_surge.grid(row=0, column=0, padx=10, pady=5)
        self.slider_surge = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_surge.grid(row=0, column=1, padx=10, pady=5)
        self.label_surge_value = tk.Label(self.master, text="0.0")
        self.label_surge_value.grid(row=0, column=2, padx=10, pady=5)

        # Sway Slider
        self.label_sway = tk.Label(self.master, text="Sway")
        self.label_sway.grid(row=1, column=0, padx=10, pady=5)
        self.slider_sway = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_sway.grid(row=1, column=1, padx=10, pady=5)
        self.label_sway_value = tk.Label(self.master, text="0.0")
        self.label_sway_value.grid(row=1, column=2, padx=10, pady=5)

        # Heave Slider
        self.label_heave = tk.Label(self.master, text="Heave")
        self.label_heave.grid(row=2, column=0, padx=10, pady=5)
        self.slider_heave = ttk.Scale(self.master, from_=-1, to=1, length=200, orient="horizontal", command=self.update_ref)
        self.slider_heave.grid(row=2, column=1, padx=10, pady=5)
        self.label_heave_value = tk.Label(self.master, text="0.0")
        self.label_heave_value.grid(row=2, column=2, padx=10, pady=5)

        # Yaw Slider
        self.label_yaw = tk.Label(self.master, text="Yaw")
        self.label_yaw.grid(row=3, column=0, padx=10, pady=5)
        self.slider_yaw = ttk.Scale(self.master, from_=-180, to=180, length=200, orient="horizontal", command=self.update_ref)
        self.slider_yaw.grid(row=3, column=1, padx=10, pady=5)
        self.label_yaw_value = tk.Label(self.master, text="0.0")
        self.label_yaw_value.grid(row=3, column=2, padx=10, pady=5)

        # Surge Offset Slider
        self.surge_offset = tk.Label(self.master, text="Surge Offset")
        self.surge_offset.grid(row=4, column=0, padx=10, pady=5)
        self.slider_surge_offset = ttk.Scale(self.master, from_=-1/2, to=1/2, length=200, orient="horizontal", command=self.update_ref)
        self.slider_surge_offset.grid(row=4, column=1, padx=10, pady=5)
        self.surge_offset_value = tk.Label(self.master, text="0.0")
        self.surge_offset_value.grid(row=4, column=2, padx=10, pady=5)

        # Sway Offset Slider
        self.sway_offset = tk.Label(self.master, text="Sway Offset")
        self.sway_offset.grid(row=5, column=0, padx=10, pady=5)
        self.slider_sway_offset = ttk.Scale(self.master, from_=-1.5, to=1.5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_sway_offset.grid(row=5, column=1, padx=10, pady=5)
        self.sway_offset_value = tk.Label(self.master, text="0.0")
        self.sway_offset_value.grid(row=5, column=2, padx=10, pady=5)

        self.ref_vals = [0.0, 0.0, 0.0, 0.0]  # Initial reference values

        self.offset_values = [0.0, 0.0]

        # Initialize ROS node and subscriber
        rospy.init_node("blueye_controller_publisher")
        self.subscriber()

    def update_ref(self, event=None):
        # Update reference values when sliders are moved
        self.ref_vals[0] = round(self.slider_surge.get(), 2)
        self.ref_vals[1] = round(self.slider_sway.get(), 2)
        self.ref_vals[2] = round(self.slider_heave.get(), 2)
        self.ref_vals[3] = round(self.slider_yaw.get(), 2)

        self.offset_values[0] = round(self.slider_surge_offset.get(), 3)
        self.offset_values[1] = round(self.slider_sway_offset.get(), 3)

        control.surge_ref = self.ref_vals[0]
        control.sway_ref = self.ref_vals[1]
        control.heave_ref = self.ref_vals[2]
        control.yaw_ref = self.ref_vals[3]

        # print(self.offset_values)

        # Update labels with current values
        self.label_surge_value.config(text=str(self.ref_vals[0]))
        self.label_sway_value.config(text=str(self.ref_vals[1]))
        self.label_heave_value.config(text=str(self.ref_vals[2]))
        self.label_yaw_value.config(text=str(self.ref_vals[3]))

        self.surge_offset_value.config(text=str(self.offset_values[0]))
        self.sway_offset_value.config(text=str(self.offset_values[1]))


    def subscriber(self):
        rospy.Subscriber("/blueye_x3/state/velocityEKF", BlueyeState, self.computeThrust)

    def computeThrust(self, msg):
        computeThrust(msg, self.ref_vals, self.offset_values)


def main():
    root = tk.Tk()
    app = BlueyeControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()