import customtkinter as ctk

# Set the application style
ctk.set_appearance_mode("Dark")  # Can also use "Light" or "System"
ctk.set_default_color_theme("blue")  # Other themes: "green", "dark-blue", etc.

app = ctk.CTk()
app.title("Blueye Interface")
app.geometry("800x500")


#####COmmands####

def dp_event():
    print("DP button Pressed")

#######


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
frame_sensors = ctk.CTkFrame(left_container)
frame_sensors.pack(fill="both", expand=True, padx=20, pady=10)

#Create a frame for Switch
switch_frame = ctk.CTkFrame(left_container)
switch_frame.pack(fill="both", expand=True, padx=20, pady=10) 


# Create a frame for the sliders
frame_sliders = ctk.CTkFrame(left_container)
frame_sliders.pack(fill="both", expand=True, padx=20, pady=10)


######################## right frames################
right_top_container = ctk.CTkFrame(right_container)
right_top_container.pack(fill="both", expand=True, padx=20, pady=10)

right_bottom_container = ctk.CTkFrame(right_container)
right_bottom_container.pack(fill="both", expand=True, padx=20, pady=10)


# Right container (for Frame 4)
data_container = ctk.CTkFrame(right_top_container)
data_container.pack(side="left", fill="both", expand=True)

pos_container = ctk.CTkFrame(data_container)
pos_container.pack(side="left", fill="both", expand=True)

vel_container = ctk.CTkFrame(data_container)
vel_container.pack(side="left", fill="both", expand=True)

# Right container (for Frame 5)
mode_container = ctk.CTkFrame(data_container)
mode_container.pack(side="left", fill="both", expand=True)
###############################################




#change these values to be updated
x_var = ctk.StringVar(value=0.0)
y_var = ctk.StringVar(value=0.0)
z_var = ctk.StringVar(value=0.0)
yaw_var = ctk.StringVar(value=0.0)
u_var = ctk.StringVar(value=0.0)
v_var = ctk.StringVar(value=0.0)
w_var = ctk.StringVar(value=0.0)
r_var = ctk.StringVar(value=0.0)

imu = False
imu_var, imu_col = (lambda: (ctk.StringVar(value="Active"), "green") if imu else (ctk.StringVar(value="Inactive"), "red"))()
dvl = True
dvl_var, dvl_col = (lambda: (ctk.StringVar(value="Active"), "green") if dvl else (ctk.StringVar(value="Inactive"), "red"))()
bar = False
bar_var, bar_col = (lambda: (ctk.StringVar(value="Active"), "green") if bar else (ctk.StringVar(value="Inactive"), "red"))()




#####LEFT CONTAINER
#FRAME ONE
#Adding Sensor Labels
ctk.CTkLabel(frame_sensors, text="SENSORS", fg_color="transparent").grid(row=0, column=0,pady=5)
ctk.CTkLabel(frame_sensors, text="IMU: ", fg_color="transparent").grid(row=1, column=0, pady=5)
ctk.CTkLabel(frame_sensors, textvariable=imu_var, text_color=imu_col).grid(row=1, column=1,pady=5)
ctk.CTkLabel(frame_sensors, text="DVL: ", fg_color="transparent").grid(row=2, column=0,pady=5)
ctk.CTkLabel(frame_sensors, textvariable=dvl_var, text_color=dvl_col).grid(row=2, column=1,pady=5)
ctk.CTkLabel(frame_sensors, text="BAR: ", fg_color="transparent").grid(row=3, column=0,pady=5)
ctk.CTkLabel(frame_sensors, textvariable=bar_var, text_color=bar_col).grid(row=3, column=1,pady=5)


#FRAME TWO
#Adding Switches
ctk.CTkLabel(switch_frame, text="SELECTORS", fg_color="transparent").grid(row=0, column=0,pady=5)

def observer_callback(choice):
    print("optionmenu dropdown clicked:", choice)

ctk.CTkLabel(switch_frame, text="Observer: ", fg_color="transparent").grid(row=1, column=0,pady=5)
observer_var = ctk.StringVar(value="Coupled")
optionmenu = ctk.CTkOptionMenu(switch_frame,values=["Coupled", "Decoupled"],
                                         command=observer_callback,
                                         variable=observer_var)

optionmenu.grid(row=1, column=1,pady=5)

def control_callback(choice):
    print("optionmenu dropdown clicked:", choice)

ctk.CTkLabel(switch_frame, text="Control Mode: ", fg_color="transparent").grid(row=2, column=0,pady=5)
control_var = ctk.StringVar(value="Velocity")
optionmenu = ctk.CTkOptionMenu(switch_frame,values=["Position", "Velocity"],
                                         command=control_callback,
                                         variable=control_var)

optionmenu.grid(row=2, column=1,pady=5)



#FRAME THREE
# Add sliders to the frame_sliders
ctk.CTkLabel(frame_sliders, text="CONTROLLERS", fg_color="transparent").grid(row=0, column=0,pady=5)

ctk.CTkSlider(frame_sliders, from_=0, to=100).grid(row=1, column=0,pady=5)
ctk.CTkSlider(frame_sliders, from_=0, to=100).grid(row=2, column=0,pady=5)
ctk.CTkSlider(frame_sliders, from_=0, to=100).grid(row=3, column=0,pady=5)
ctk.CTkButton(frame_sliders, text="START", fg_color="green").grid(row=4, column=0,pady=5)
# ctk.CTkButton(frame_sliders, text="STOP", fg_color="red").grid(row=4, column=1,pady=5)


####RIGHT CONTAINER

#FRAME 4
ctk.CTkLabel(pos_container, text="POSITION", fg_color="transparent").grid(row=0, column=0,pady=5)

ctk.CTkLabel(pos_container, text="X: ", fg_color="transparent").grid(row=1, column=0,pady=5)
ctk.CTkLabel(pos_container, textvariable=x_var, fg_color="transparent").grid(row=1, column=1,pady=5)
ctk.CTkLabel(pos_container, text="Y: ", fg_color="transparent").grid(row=2, column=0,pady=5)
ctk.CTkLabel(pos_container, textvariable=y_var, fg_color="transparent").grid(row=2, column=1,pady=5)
ctk.CTkLabel(pos_container, text="Z: ", fg_color="transparent").grid(row=3, column=0,pady=5)
ctk.CTkLabel(pos_container, textvariable=z_var, fg_color="transparent").grid(row=3, column=1,pady=5)
ctk.CTkLabel(pos_container, text="Yaw: ", fg_color="transparent").grid(row=4, column=0,pady=5)
ctk.CTkLabel(pos_container, textvariable=yaw_var, fg_color="transparent").grid(row=4, column=1,pady=5)


ctk.CTkLabel(vel_container, text="VELOCITY", fg_color="transparent").grid(row=0, column=0,pady=5)

ctk.CTkLabel(vel_container, text="U: ", fg_color="transparent").grid(row=1, column=0,pady=5)
ctk.CTkLabel(vel_container, textvariable=u_var, fg_color="transparent").grid(row=1, column=1,pady=5)
ctk.CTkLabel(vel_container, text="V: ", fg_color="transparent").grid(row=2, column=0,pady=5)
ctk.CTkLabel(vel_container, textvariable=v_var, fg_color="transparent").grid(row=2, column=1,pady=5)
ctk.CTkLabel(vel_container, text="W: ", fg_color="transparent").grid(row=3, column=0,pady=5)
ctk.CTkLabel(vel_container, textvariable=w_var, fg_color="transparent").grid(row=3, column=1,pady=5)
ctk.CTkLabel(vel_container, text="R: ", fg_color="transparent").grid(row=4, column=0,pady=5)
ctk.CTkLabel(vel_container, textvariable=r_var, fg_color="transparent").grid(row=4, column=1,pady=5)


#FRAME 5
ctk.CTkLabel(mode_container, text="MODES", fg_color="transparent").grid(row=0, column=0,pady=5)
ctk.CTkButton(mode_container, text="DP", fg_color="blue", command=dp_event).grid(row=1, column=0,pady=5)
ctk.CTkButton(mode_container, text="CIRCLE", fg_color="blue").grid(row=2, column=0,pady=5)
ctk.CTkButton(mode_container, text="ASCEND", fg_color="blue").grid(row=3, column=0,pady=5)
ctk.CTkButton(mode_container, text="DESCEND", fg_color="blue").grid(row=4, column=0,pady=5)
# ctk.CTkButton(mode_container, text="SQUARE", fg_color="blue").grid(row=5, column=0,pady=5)





app.mainloop()

#################################


# import customtkinter as ctk

# app = ctk.CTk()
# app.geometry("600x400")

# # Main container frame
# main_container = ctk.CTkFrame(app)
# main_container.pack(fill="both", expand=True)

# # Left container (for Frame 1, Frame 2, and Frame 3)
# left_container = ctk.CTkFrame(main_container)
# left_container.pack(side="left", fill="both", expand=True)

# # Right container (for Frame 4)
# right_container = ctk.CTkFrame(main_container)
# right_container.pack(side="left", fill="both", expand=True)

# # Frames 1, 2, 3 in the left container
# frame1 = ctk.CTkFrame(left_container, height=100, fg_color="gray")
# frame1.pack(pady=10, padx=10, fill="x")
# label1 = ctk.CTkLabel(frame1, text="Frame 1")
# label1.pack(pady=10)

# frame2 = ctk.CTkFrame(left_container, height=100, fg_color="gray")
# frame2.pack(pady=10, padx=10, fill="x")
# label2 = ctk.CTkLabel(frame2, text="Frame 2")
# label2.pack(pady=10)

# frame3 = ctk.CTkFrame(left_container, height=100, fg_color="gray")
# frame3.pack(pady=10, padx=10, fill="x")
# label3 = ctk.CTkLabel(frame3, text="Frame 3")
# label3.pack(pady=10)

# # Frame 4 in the right container
# frame4 = ctk.CTkFrame(right_container, fg_color="gray")
# frame4.pack(pady=10, padx=10, fill="both", expand=True)
# label4 = ctk.CTkLabel(frame4, text="Frame 4")
# label4.pack(pady=10)

# app.mainloop()


#######

# import customtkinter as ctk
# import tkinter as tk

# # Function to simulate sensor status checking
# def get_sensor_status(sensor_name):
#     # Simulate sensor status for demonstration purposes
#     import random
#     return "Active" if random.choice([True, False]) else "Inactive"

# # Update sensor status in the GUI
# def update_sensor_status():
#     imu_status_frame.configure(fg_color=("green" if imu_status_label.cget("text") == "Active" else "red"))
#     dvl_status_frame.configure(fg_color=("green" if dvl_status_label.cget("text") == "Active" else "red"))
#     bar_status_frame.configure(fg_color=("green" if bar_status_label.cget("text") == "Active" else "red"))
#     imu_status_label.configure(text=get_sensor_status('imu'))
#     dvl_status_label.configure(text=get_sensor_status('dvl'))
#     bar_status_label.configure(text=get_sensor_status('bar'))
#     app.after(1000, update_sensor_status)

# app = ctk.CTk()
# app.title("Sensor Status Monitor")
# app.geometry("400x200")

# # Create a frame for the sensors
# frame_sensors = ctk.CTkFrame(app, corner_radius=10)
# frame_sensors.pack(pady=20, padx=20, fill="both", expand=True)

# # Sensor status title label
# title_label = ctk.CTkLabel(frame_sensors, text="SENSORS")
# title_label.pack(pady=(10, 20))

# # Function to create sensor labels with separate status frames
# def create_sensor_label(frame, text):
#     label_frame = ctk.CTkFrame(frame, corner_radius=10)
#     sensor_label = ctk.CTkLabel(label_frame, text=text)
#     status_label = ctk.CTkLabel(label_frame, text="Checking...")
#     sensor_label.pack(side="left")
#     status_label.pack(side="right", padx=10)
#     label_frame.pack(pady=5, fill="x", padx=20)
#     return label_frame, status_label

# # Create labels for each sensor
# imu_status_frame, imu_status_label = create_sensor_label(frame_sensors, "IMU")
# dvl_status_frame, dvl_status_label = create_sensor_label(frame_sensors, "DVL")
# bar_status_frame, bar_status_label = create_sensor_label(frame_sensors, "BAR")

# # Initial update of sensor statuses
# update_sensor_status()

# app.mainloop()

