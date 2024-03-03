#!/home/aduragbemi/.pyenv/shims/python
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForceRef
from blueye.sdk import Drone
import numpy as np

import tkinter as tk
from tkinter import ttk

class ObserverGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Blueye Drone Observer Tuner")

        # Pmc1 Slider pnc and mnc
        self.label_pnc1 = tk.Label(self.master, text="Pnc1")
        self.label_pnc1.grid(row=0, column=0, padx=10, pady=5)
        self.slider_pnc1 = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_pnc1.grid(row=0, column=1, padx=10, pady=5)
        self.label_pnc1_value = tk.Label(self.master, text="0.0")
        self.label_pnc1_value.grid(row=0, column=2, padx=10, pady=5)

        # Pmc2 Slider pnc and mnc
        self.label_pnc2 = tk.Label(self.master, text="Pnc2")
        self.label_pnc2.grid(row=1, column=0, padx=10, pady=5)
        self.slider_pnc2 = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_pnc2.grid(row=1, column=1, padx=10, pady=5)
        self.label_pnc2_value = tk.Label(self.master, text="0.0")
        self.label_pnc2_value.grid(row=1, column=2, padx=10, pady=5)

        # Pmc3 Slider pnc and mnc
        self.label_pnc3 = tk.Label(self.master, text="Pnc3")
        self.label_pnc3.grid(row=2, column=0, padx=10, pady=5)
        self.slider_pnc3 = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_pnc3.grid(row=2, column=1, padx=10, pady=5)
        self.label_pnc3_value = tk.Label(self.master, text="0.0")
        self.label_pnc3_value.grid(row=2, column=2, padx=10, pady=5)

        # mnc1 Slider
        self.label_mnc1 = tk.Label(self.master, text="Mnc1")
        self.label_mnc1.grid(row=3, column=0, padx=10, pady=5)
        self.slider_mnc1 = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_mnc1.grid(row=3, column=1, padx=10, pady=5)
        self.label_mnc1_value = tk.Label(self.master, text="0.0")
        self.label_mnc1_value.grid(row=3, column=2, padx=10, pady=5)

        # mnc2 Slider
        self.label_mnc2 = tk.Label(self.master, text="Mnc2")
        self.label_mnc2.grid(row=4, column=0, padx=10, pady=5)
        self.slider_mnc2 = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_mnc2.grid(row=4, column=1, padx=10, pady=5)
        self.label_mnc2_value = tk.Label(self.master, text="0.0")
        self.label_mnc2_value.grid(row=4, column=2, padx=10, pady=5)

        # mnc3 Slider
        self.label_mnc3 = tk.Label(self.master, text="Mnc3")
        self.label_mnc3.grid(row=5, column=0, padx=10, pady=5)
        self.slider_mnc3 = ttk.Scale(self.master, from_=-5, to=5, length=200, orient="horizontal", command=self.update_ref)
        self.slider_mnc3.grid(row=5, column=1, padx=10, pady=5)
        self.label_mnc3_value = tk.Label(self.master, text="0.0")
        self.label_mnc3_value.grid(row=5, column=2, padx=10, pady=5)

        # # Heave Slider
        # self.label_heave = tk.Label(self.master, text="Heave")
        # self.label_heave.grid(row=2, column=0, padx=10, pady=5)
        # self.slider_heave = ttk.Scale(self.master, from_=-1, to=9, length=200, orient="horizontal", command=self.update_ref)
        # self.slider_heave.grid(row=2, column=1, padx=10, pady=5)
        # self.label_heave_value = tk.Label(self.master, text="0.0")
        # self.label_heave_value.grid(row=2, column=2, padx=10, pady=5)

        # # Yaw Slider
        # self.label_yaw = tk.Label(self.master, text="Yaw")
        # self.label_yaw.grid(row=3, column=0, padx=10, pady=5)
        # self.slider_yaw = ttk.Scale(self.master, from_=-180, to=180, length=200, orient="horizontal", command=self.update_ref)
        # self.slider_yaw.grid(row=3, column=1, padx=10, pady=5)
        # self.label_yaw_value = tk.Label(self.master, text="0.0")
        # self.label_yaw_value.grid(row=3, column=2, padx=10, pady=5)

        self.tune_vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial reference values

        # Initialize ROS node and subscriber
        # rospy.init_node("blueye_controller_publisher")
        self.computeThrust()

    def update_ref(self, event=None):
        # Update reference values when sliders are moved
        self.tune_vals[0] = round(self.slider_pnc1.get(), 2)
        self.tune_vals[1] = round(self.slider_pnc2.get(), 2)
        self.tune_vals[2] = round(self.slider_pnc3.get(), 2)

        self.tune_vals[3] = round(self.slider_mnc1.get(), 2)
        self.tune_vals[4] = round(self.slider_mnc2.get(), 2)
        self.tune_vals[5] = round(self.slider_mnc3.get(), 2)

        # self.tune_vals[2] = round(self.slider_heave.get(), 2)
        # self.tune_vals[3] = round(self.slider_yaw.get(), 2)

        print(self.tune_vals)

        # Update labels with current values
        self.label_pnc1_value.config(text=str(self.tune_vals[0]))
        self.label_pnc2_value.config(text=str(self.tune_vals[1]))
        self.label_pnc3_value.config(text=str(self.tune_vals[2]))

        self.label_mnc1_value.config(text=str(self.tune_vals[3]))
        self.label_mnc2_value.config(text=str(self.tune_vals[4]))
        self.label_mnc3_value.config(text=str(self.tune_vals[5]))

        # return self.tune_vals

        # self.label_heave_value.config(text=str(self.tune_vals[2]))
        # self.label_yaw_value.config(text=str(self.tune_vals[3]))

    # def subscriber(self):
    #     rospy.Subscriber("/blueye_x3/state/EKF_coupled", BlueyeState, self.computeThrust)

    def computeThrust(self):
        return self.tune_vals
        # computeThrust(msg, self.tune_vals)

def main():
    root = tk.Tk()
    app = ObserverGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()