#!/usr/bin/env python3
import rospy
from blueye_x3_ros.msg import BlueyeState, BlueyeForceRef, BlueyeImu, BlueyeDepth
from waterlinked_a50_ros_driver.msg import DVL
from blueye.sdk import Drone
import numpy as np
from PID import PID



import tkinter as tk
import customtkinter as ctk
from tkinter import ttk


app = ctk.CTk()
app.title("Blueye Interface")
app.geometry("950x800")

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("blue")

# Main container frame
main_container = ctk.CTkFrame(app)
main_container.pack(fill="both", expand=True)

#####################################
# Left container (for Frame 1, Frame 2, and Frame 3)
left_container = ctk.CTkFrame(main_container)
left_container.pack(side="left", fill="both", expand=True)

# Right container (for Frame 4)
right_container = ctk.CTkFrame(main_container)
right_container.pack(side="left", fill="both", expand=True)

###################################


################################## left frames ################
left_top_container = ctk.CTkFrame(left_container)
left_top_container.pack(fill="both", expand=True, padx=20, pady=10)

data_container = ctk.CTkFrame(left_top_container)
data_container.pack(side="left", fill="both", expand=True)

sensors_frame = ctk.CTkFrame(data_container)
sensors_frame.pack(side="left", fill="both", expand=True)

pos_container = ctk.CTkFrame(data_container)
pos_container.pack(side="left", fill="both", expand=True)

vel_container = ctk.CTkFrame(data_container)
vel_container.pack(side="left", fill="both", expand=True)

#Create a frame for Switch
switch_frame = ctk.CTkFrame(left_container)
switch_frame.pack(fill="both", expand=True, padx=20, pady=10) 


# Create a frame for the sliders
sliders_frame = ctk.CTkFrame(left_container)
sliders_frame.pack(fill="both", expand=True, padx=20, pady=10)


######################## right frames################
right_top_container = ctk.CTkFrame(right_container)
right_top_container.pack(fill="both", expand=True, padx=20, pady=10)

right_bottom_container = ctk.CTkFrame(right_container)
right_bottom_container.pack(fill="both", expand=True, padx=20, pady=10)


# Right container (for Frame 4)
mode_container = ctk.CTkFrame(right_top_container)
mode_container.pack(side="left", fill="both", expand=True)

# pos_container = ctk.CTkFrame(data_container)
# pos_container.pack(side="left", fill="both", expand=True)

# vel_container = ctk.CTkFrame(data_container)
# vel_container.pack(side="left", fill="both", expand=True)

# Right container (for Frame 5)
mode_button_container = ctk.CTkFrame(mode_container)
mode_button_container.pack(side="left", fill="both", expand=True)

mode_slider_container = ctk.CTkFrame(mode_container)
mode_slider_container.pack(side="left", fill="both", expand=True)
###############################################







start_ctrl = False
mode = "Velocity"
thrust = "Inactive"
init_ref = True
ref_value = [0.0, 0.0, 0.0, 0.0]













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


def normalize_angle(angle):
    """ Normalize an angle to [-180, 180] interval """
    normalized_angle = np.deg2rad((angle + 180) % 360 - 180)
    return normalized_angle


def computeThrust(msg, ref_vals, offset_vals, mode_vals):
    # print(ref_vals)
    global start_ctrl, mode
    global init_ref, ref_value


    if start_ctrl:
        Kp = np.array([1.3, 3.0, 1.8, 0.5]) #TUNE x, y, z, yaw 0.5 0.3 1.8 0.4 ---u,v 0.03, 0.0005
        Ki = np.array([0.0, 0.05, 0.3, 0.07]) #TUNE 0.2 0.1 0.3 0.09 --- 0.06, 0.002
        Kd = np.array([0.1, 1.0, 0.5, 0.3]) #TUNE 0 0 0.5 0.4 -----0.001, 0.001
        # rospy.loginfo(ref_vals) 

        if mode == "Velocity":
            ref_value = [ref_vals[0],ref_vals[1],ref_vals[3],ref_vals[5]]
            control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
            # pos_PID = PID(setpoint=[ref_vals[0],ref_vals[1],ref_vals[3],ref_vals[5]], Kp=Kp, Ki=Ki, Kd=Kd)
            feedback_value = np.array([msg.u, msg.v, msg.w, msg.r]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)


        #positioning using u, v, z, psi
        # elif mode == "Position":
        #     ref_value = [ref_vals[0], ref_vals[1], ref_vals[2], normalize_angle(ref_vals[4])]
        #     control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
        #     # pos_PID = PID(setpoint=[ref_vals[0],ref_vals[1],ref_vals[2],normalize_angle(ref_vals[4])], Kp=Kp, Ki=Ki, Kd=Kd)
        #     feedback_value = np.array([msg.u, msg.v, msg.z, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)


        # # #Full positioning x, y, z, psi
        elif mode == "Position":
            if init_ref:
                ref_value = [msg.x, msg.y, ref_vals[2], normalize_angle(ref_vals[4])]
                control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
                init_ref = False
            ref_value = ref_value
            # pos_PID = PID(setpoint=[ref_vals[0],ref_vals[1],ref_vals[2],normalize_angle(ref_vals[4])], Kp=Kp, Ki=Ki, Kd=Kd)
            feedback_value = np.array([msg.x, msg.y, msg.z, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)

        elif mode=="Station":
            if init_ref:
                ref_value = [0, 0, msg.z, normalize_angle(msg.psi)]
                control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
                init_ref = False
            ref_value = ref_value
                
            feedback_value = np.array([msg.u, msg.v, msg.z, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)

        elif mode=="Rotate":
            circle_speed = mode_vals[0]
            if init_ref:
                # depth = msg.z
                ref_value = [0, 0, msg.z, circle_speed]
                # pos_PID = PID(setpoint=[0,0,msg.z,circle_speed], Kp=Kp, Ki=Ki, Kd=Kd)
                init_ref = False
            ref_value[3] = circle_speed
            control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
            feedback_value = np.array([msg.u, msg.v, msg.z, msg.r]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)

        elif mode=="Ascend":
            ascend_speed = - mode_vals[1]
            if init_ref:
                ref_value = [0, 0, ascend_speed, normalize_angle(msg.psi)]
                # pos_PID = PID(setpoint=[0, 0, ascend_speed, normalize_angle(msg.psi)], Kp=Kp, Ki=Ki, Kd=Kd)
                init_ref = False
            ref_value[2] = ascend_speed
            control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
            feedback_value = np.array([msg.u, msg.v, msg.w, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)

        elif mode=="Descend":
            descend_speed = mode_vals[2]
            if init_ref:
                ref_value = [0, 0, descend_speed, normalize_angle(msg.psi)]
                init_ref = False
            ref_value[2] = descend_speed
            control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
            feedback_value = np.array([msg.u, msg.v, msg.w, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)

        # elif mode=="Square":
        #     if init_ref:
        #         square_speed = mode_vals[3]
        #         ref_value = [square_speed, square_speed, msg.z, normalize_angle(msg.psi)]
        #         control.surge_ref, control.sway_ref, control.heave_ref, control.yaw_ref = ref_value
        #         init_ref = False
        #     feedback_value = np.array([msg.u, msg.v, msg.z, normalize_angle(msg.psi)]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)
            
            # pos_PID = PID(setpoint=[ref_vals[0],ref_vals[1],ref_vals[3],ref_vals[5]], Kp=Kp, Ki=Ki, Kd=Kd)
            # feedback_value = np.array([msg.u, msg.v, msg.w, msg.r]) #msg.x, msg.y, msg.z, np.deg2rad(msg.psi)


        pos_PID = PID(setpoint= ref_value, Kp=Kp, Ki=Ki, Kd=Kd)
        output = pos_PID.compute(feedback_value)
        # print(output)
        writeThrustValues(surge=output[0], sway=output[1], heave=output[2], yaw=output[3], offset_vals=offset_vals)
        # writeThrustValues(surge=0, sway=0, heave=-0.3, yaw=output[3])

    else:
        pass


def start_event():
    global start_ctrl, thrust
    # print("start button pressed")
    start_ctrl = True
    thrust = "Active"


def stop_event():
    global start_ctrl, thrust
    start_ctrl = False
    thrust = "Inactive"
    # print("stop button pressed")

def dp_mode():
    global mode, init_ref
    init_ref = True
    mode = "Station"
    # print("dp button pressed")
    pass

def circle_mode():
    global mode, init_ref
    init_ref = True
    mode = "Rotate"
    # print("circle button pressed")
    pass

def ascend_mode():
    global mode, init_ref
    init_ref = True
    mode = "Ascend"
    # print("ascend button pressed")
    pass

def descend_mode():
    global mode, init_ref
    init_ref = True
    mode = "Descend"
    # print("descend button pressed")
    pass

def square_mode():
    global mode, init_ref
    init_ref = True
    mode = "Square"
    # print("square button pressed")
    pass

class BlueyeControllerGUI:
    def __init__(self, master):
        self.master = master
        # self.master.title("Blueye Drone Reference Controller")

        self.current_topic = "/blueye_x3/state/EKF_coupled"

        # self.control_sw = True #True for velocity control and False for position control



        #####LEFT CONTAINER
        #FRAME ONE
        #Adding Sensor Labels


        ctk.CTkLabel(sensors_frame, text="SENSORS", fg_color="transparent").grid(row=0, column=0,pady=5)
        ctk.CTkLabel(sensors_frame, text="IMU: ", fg_color="transparent").grid(row=1, column=0, pady=5)
        self.imu_status = ctk.CTkLabel(sensors_frame, text="Inactive", text_color="red")
        self.imu_status.grid(row=1, column=1,pady=5)
        ctk.CTkLabel(sensors_frame, text="DVL: ", fg_color="transparent").grid(row=2, column=0,pady=5)
        self.dvl_status = ctk.CTkLabel(sensors_frame, text="Inactive", text_color="red")
        self.dvl_status.grid(row=2, column=1,pady=5)
        ctk.CTkLabel(sensors_frame, text="BAR: ", fg_color="transparent").grid(row=3, column=0,pady=5)
        self.bar_status = ctk.CTkLabel(sensors_frame, text="Inactive", text_color="red")
        self.bar_status.grid(row=3, column=1,pady=5)
        ctk.CTkLabel(sensors_frame, text="MODE: ", fg_color="transparent").grid(row=4, column=0,pady=5)
        self.mode_status = ctk.CTkLabel(sensors_frame, text="Loading...", text_color="yellow")
        self.mode_status.grid(row=4, column=1,pady=5)
        ctk.CTkLabel(sensors_frame, text="STATUS: ", fg_color="transparent").grid(row=5, column=0,pady=5)
        self.thrust_status = ctk.CTkLabel(sensors_frame, text="Inactive", text_color="red")
        self.thrust_status.grid(row=5, column=1,pady=5)


        #FRAME TWO
        #Adding Switches
        ctk.CTkLabel(switch_frame, text="SELECTORS", fg_color="transparent").grid(row=0, column=0,pady=5)


        ctk.CTkLabel(switch_frame, text="Observer: ", fg_color="transparent").grid(row=1, column=0,pady=5)
        # self.observer_var = ctk.StringVar(value="Coupled")
        ctk.CTkOptionMenu(switch_frame,values=["Coupled", "Decoupled"],
                                                command=self.model_switch_event).grid(row=1, column=1,pady=5)


        ctk.CTkLabel(switch_frame, text="Control Mode: ", fg_color="transparent").grid(row=2, column=0,pady=5)
        # self.control_var = ctk.StringVar(value="Velocity")
        ctk.CTkOptionMenu(switch_frame,values=["Velocity", "Position"],
                                                command=self.control_switch_event).grid(row=2, column=1,pady=5)



        #FRAME THREE
        # Add sliders to the sliders_frame
        ctk.CTkLabel(sliders_frame, text="CONTROLLERS", fg_color="transparent").grid(row=0, column=0,pady=5)



        # Surge Slider
        self.label_surge = ctk.CTkLabel(sliders_frame, text="Surge")
        self.label_surge.grid(row=1, column=0, padx=10, pady=5)
        self.slider_surge = ctk.CTkSlider(sliders_frame, from_=-0.5, to=0.5, orientation="horizontal", command=self.update_ref)
        self.slider_surge.grid(row=1, column=1, padx=10, pady=5)
        self.label_surge_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.label_surge_value.grid(row=1, column=2, padx=10, pady=5)

        # Sway Slider
        self.label_sway = ctk.CTkLabel(sliders_frame, text="Sway")
        self.label_sway.grid(row=2, column=0, padx=10, pady=5)
        self.slider_sway = ctk.CTkSlider(sliders_frame, from_=-0.5, to=0.5, orientation="horizontal", command=self.update_ref)
        self.slider_sway.grid(row=2, column=1, padx=10, pady=5)
        self.label_sway_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.label_sway_value.grid(row=2, column=2, padx=10, pady=5)

        # Heave Slider
        self.label_heave = ctk.CTkLabel(sliders_frame, text="Heave")
        self.label_heave.grid(row=3, column=0, padx=10, pady=5)
        self.slider_heave = ctk.CTkSlider(sliders_frame, from_=-1, to=1, orientation="horizontal", command=self.update_ref)
        self.slider_heave.grid(row=3, column=1, padx=10, pady=5)
        self.label_heave_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.label_heave_value.grid(row=3, column=2, padx=10, pady=5)

        # Heave rate Slider
        self.label_heave_rate = ctk.CTkLabel(sliders_frame, text="Heave_Rate")
        self.label_heave_rate.grid(row=4, column=0, padx=10, pady=5)
        self.slider_heave_rate = ctk.CTkSlider(sliders_frame, from_=-1, to=1, orientation="horizontal", command=self.update_ref)
        self.slider_heave_rate.grid(row=4, column=1, padx=10, pady=5)
        self.label_heave_rate_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.label_heave_rate_value.grid(row=4, column=2, padx=10, pady=5)

        # Yaw Slider
        self.label_yaw = ctk.CTkLabel(sliders_frame, text="Yaw")
        self.label_yaw.grid(row=5, column=0, padx=10, pady=5)
        self.slider_yaw = ctk.CTkSlider(sliders_frame, from_=0, to=360, orientation="horizontal", command=self.update_ref)
        self.slider_yaw.grid(row=5, column=1, padx=10, pady=5)
        self.label_yaw_value = ctk.CTkLabel(sliders_frame, text="180.0")
        self.label_yaw_value.grid(row=5, column=2, padx=10, pady=5)

        # Yaw rate Slider
        self.label_yaw_rate = ctk.CTkLabel(sliders_frame, text="Yaw_Rate")
        self.label_yaw_rate.grid(row=6, column=0, padx=10, pady=5)
        self.slider_yaw_rate = ctk.CTkSlider(sliders_frame, from_=-1, to=1, orientation="horizontal", command=self.update_ref)
        self.slider_yaw_rate.grid(row=6, column=1, padx=10, pady=5)
        self.label_yaw_rate_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.label_yaw_rate_value.grid(row=6, column=2, padx=10, pady=5)

        # Surge Offset Slider
        self.surge_offset = ctk.CTkLabel(sliders_frame, text="Surge Offset")
        self.surge_offset.grid(row=7, column=0, padx=10, pady=5)
        self.slider_surge_offset = ctk.CTkSlider(sliders_frame, from_=-1/2, to=1/2, orientation="horizontal", command=self.update_ref)
        self.slider_surge_offset.grid(row=7, column=1, padx=10, pady=5)
        self.surge_offset_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.surge_offset_value.grid(row=7, column=2, padx=10, pady=5)

        # Sway Offset Slider
        self.sway_offset = ctk.CTkLabel(sliders_frame, text="Sway Offset")
        self.sway_offset.grid(row=8, column=0, padx=10, pady=5)
        self.slider_sway_offset = ctk.CTkSlider(sliders_frame, from_=-1.5, to=1.5, orientation="horizontal", command=self.update_ref)
        self.slider_sway_offset.grid(row=8, column=1, padx=10, pady=5)
        self.sway_offset_value = ctk.CTkLabel(sliders_frame, text="0.0")
        self.sway_offset_value.grid(row=8, column=2, padx=10, pady=5)

        ctk.CTkButton(sliders_frame, text="START", fg_color="green", command=start_event).grid(row=9, column=0, pady=5, padx=10)
        ctk.CTkButton(sliders_frame, text="STOP", fg_color="red", command=stop_event).grid(row=9, column=1,pady=5)


        ####RIGHT CONTAINER

        #FRAME 4
        ctk.CTkLabel(pos_container, text="POSITION", fg_color="transparent").grid(row=0, column=0,pady=5)

        ctk.CTkLabel(pos_container, text="X: ", fg_color="transparent").grid(row=1, column=0,pady=5)
        self.x_status = ctk.CTkLabel(pos_container, text="0.0", fg_color="transparent")
        self.x_status.grid(row=1, column=1,pady=5)
        ctk.CTkLabel(pos_container, text="Y: ", fg_color="transparent").grid(row=2, column=0,pady=5)
        self.y_status = ctk.CTkLabel(pos_container, text="0.0", fg_color="transparent")
        self.y_status.grid(row=2, column=1,pady=5)
        ctk.CTkLabel(pos_container, text="Z: ", fg_color="transparent").grid(row=3, column=0,pady=5)
        self.z_status = ctk.CTkLabel(pos_container, text="0.0", fg_color="transparent")
        self.z_status.grid(row=3, column=1,pady=5)
        ctk.CTkLabel(pos_container, text="Yaw: ", fg_color="transparent").grid(row=4, column=0,pady=5)
        self.yaw_status = ctk.CTkLabel(pos_container, text="0.0", fg_color="transparent")
        self.yaw_status.grid(row=4, column=1,pady=5)


        ctk.CTkLabel(vel_container, text="VELOCITY", fg_color="transparent").grid(row=0, column=0,pady=5)

        ctk.CTkLabel(vel_container, text="U: ", fg_color="transparent").grid(row=1, column=0,pady=5)
        self.u_status = ctk.CTkLabel(vel_container, text="0.0", fg_color="transparent")
        self.u_status.grid(row=1, column=1,pady=5)
        ctk.CTkLabel(vel_container, text="V: ", fg_color="transparent").grid(row=2, column=0,pady=5)
        self.v_status = ctk.CTkLabel(vel_container, text="0.0", fg_color="transparent")
        self.v_status.grid(row=2, column=1,pady=5)
        ctk.CTkLabel(vel_container, text="W: ", fg_color="transparent").grid(row=3, column=0,pady=5)
        self.w_status = ctk.CTkLabel(vel_container, text="0.0", fg_color="transparent")
        self.w_status.grid(row=3, column=1,pady=5)
        ctk.CTkLabel(vel_container, text="R: ", fg_color="transparent").grid(row=4, column=0,pady=5)
        self.r_status = ctk.CTkLabel(vel_container, text="0.0", fg_color="transparent")
        self.r_status.grid(row=4, column=1,pady=5)


        #FRAME 5


        # self.label_surge = ctk.CTkLabel(sliders_frame, text="Surge")
        # self.label_surge.grid(row=1, column=0, padx=10, pady=5)
        # self.slider_surge = ctk.CTkSlider(sliders_frame, from_=-0.5, to=0.5, orientation="horizontal", command=self.update_ref)
        # self.slider_surge.grid(row=1, column=1, padx=10, pady=5)
        # self.label_surge_value = ctk.CTkLabel(sliders_frame, text="0.0")
        # self.label_surge_value.grid(row=1, column=2, padx=10, pady=5)



        ctk.CTkLabel(mode_button_container, text="MODES", fg_color="transparent").grid(row=0, column=0,pady=5)
        ctk.CTkButton(mode_button_container, text="STATION KEEP", fg_color="blue", command=dp_mode).grid(row=1, column=0,pady=5)

        ctk.CTkButton(mode_button_container, text="ROTATE", fg_color="blue", command=circle_mode).grid(row=2, column=0,pady=5, padx=10)
        self.slider_rotate = ctk.CTkSlider(mode_button_container, from_=-1.0, to=1.0, orientation="horizontal", command=self.update_ref)
        self.slider_rotate.grid(row=2, column=1, padx=10, pady=5)
        self.label_rotate_value = ctk.CTkLabel(mode_button_container, text="0.0")
        self.label_rotate_value.grid(row=2, column=2, padx=10, pady=5)


        ctk.CTkButton(mode_button_container, text="ASCEND", fg_color="blue", command=ascend_mode).grid(row=3, column=0,pady=5)
        self.slider_ascend = ctk.CTkSlider(mode_button_container, from_=0.0, to=1.0, orientation="horizontal", command=self.update_ref)
        self.slider_ascend.grid(row=3, column=1, padx=10, pady=5)
        self.label_ascend_value = ctk.CTkLabel(mode_button_container, text="0.2")
        self.label_ascend_value.grid(row=3, column=2, padx=10, pady=5)

        ctk.CTkButton(mode_button_container, text="DESCEND", fg_color="blue", command=descend_mode).grid(row=4, column=0,pady=5)
        self.slider_descend = ctk.CTkSlider(mode_button_container, from_=0.0, to=1.0, orientation="horizontal", command=self.update_ref)
        self.slider_descend.grid(row=4, column=1, padx=10, pady=5)
        self.label_descend_value = ctk.CTkLabel(mode_button_container, text="0.2")
        self.label_descend_value.grid(row=4, column=2, padx=10, pady=5)

        # ctk.CTkButton(mode_button_container, text="SQUARE", fg_color="blue", command=square_mode).grid(row=5, column=0,pady=5)
        # self.slider_square = ctk.CTkSlider(mode_button_container, from_=-1.0, to=1.0, orientation="horizontal", command=self.update_ref)
        # self.slider_square.grid(row=5, column=1, padx=10, pady=5)
        # self.label_square_value = ctk.CTkLabel(mode_button_container, text="0.0")
        # self.label_square_value.grid(row=5, column=2, padx=10, pady=5)


        

        self.ref_vals = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial reference values

        self.offset_values = [0.0, 0.0]

        self.mode_vals = [0.0, 0.2, 0.2, 0.0]


        # Initialize ROS node and subscriber
        rospy.init_node("blueye_controller_publisher")


        rospy.Subscriber("/blueye_x3/imu", BlueyeImu, self.imuCheck)
        rospy.Subscriber("/blueye_x3/depth", BlueyeDepth, self.depthCheck)
        rospy.Subscriber("/dvl/data", DVL, self.dvlCheck)

        self.subscriber = rospy.Subscriber(self.current_topic, BlueyeState, self.computeThrust)

        # self.subscriber()


    def imuCheck(self, msg):
        status, color = "Active", "green"
        self.master.after(0, lambda: self.imu_status.configure(text=status, text_color=color))

    def depthCheck(self, msg):
        status, color = "Active", "green"
        self.master.after(0, lambda: self.bar_status.configure(text=status, text_color=color))

    def dvlCheck(self, msg):
        status, color = "Active", "green"
        self.master.after(0, lambda: self.dvl_status.configure(text=status, text_color=color))

    def model_switch_event(self, observer_var):
        # print(observer_var)
        if observer_var == "Coupled":
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

    def control_switch_event(self, control_var):
        global mode, init_ref
        # print(self.control_selector.get())
        # print(control_var)
        if control_var == "Velocity":
            mode = "Velocity"
        else:
            mode = "Position"
            # init_ref = True
        

            

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


        self.mode_vals[0] = round(self.slider_rotate.get(), 2)
        self.mode_vals[1] = round(self.slider_ascend.get(), 2)
        self.mode_vals[2] = round(self.slider_descend.get(), 2)
        # self.mode_vals[3] = round(self.slider_square.get(), 2)
        

        # control.surge_ref = self.ref_vals[0]
        # control.sway_ref = self.ref_vals[1]
        # control.heave_ref = self.ref_vals[2]
        # control.yaw_ref = self.ref_vals[4]

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


        self.label_rotate_value.configure(text=str(self.mode_vals[0]))
        self.label_ascend_value.configure(text=str(self.mode_vals[1]))
        self.label_descend_value.configure(text=str(self.mode_vals[2]))
        # self.label_square_value.configure(text=str(self.mode_vals[3]))


    # def subscriber(self):
    #     # rospy.Subscriber("/blueye_x3/state/velocityEKF", BlueyeState, self.computeThrust)
    #     print(self.topic_name)
    #     rospy.Subscriber(self.topic_name, BlueyeState, self.computeThrust)

    def computeThrust(self, msg):
        global thrust
        self.master.after(0, lambda: self.x_status.configure(text=round(msg.x , 2)))
        self.master.after(0, lambda: self.y_status.configure(text=round(msg.y , 2)))
        self.master.after(0, lambda: self.z_status.configure(text=round(msg.z , 4)))
        self.master.after(0, lambda: self.yaw_status.configure(text=round(msg.psi , 2)))
        self.master.after(0, lambda: self.u_status.configure(text=round(msg.u , 4)))
        self.master.after(0, lambda: self.v_status.configure(text=round(msg.v , 4)))
        self.master.after(0, lambda: self.w_status.configure(text=round(msg.w , 4)))
        self.master.after(0, lambda: self.r_status.configure(text=round(msg.r , 4)))
        self.master.after(0, lambda: self.mode_status.configure(text=mode, text_color="green"))

        if thrust == "Active":
            self.master.after(0, lambda: self.thrust_status.configure(text=thrust, text_color="green"))
        else:
            self.master.after(0, lambda: self.thrust_status.configure(text=thrust, text_color="red"))
        computeThrust(msg, self.ref_vals, self.offset_values, self.mode_vals)


def main():
    # root = ctk.CTk()
    # root.geometry("720x480")

    # app = ctk.CTk()
    # app.title("Blueye Interface")
    # app.geometry("800x500")

    BlueyeControllerGUI(app)
    app.mainloop()

if __name__ == "__main__":
    main()