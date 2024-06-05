# blueye_x3_ros by Aduragbemi Adetunji. For more information about use, contact at adetunjiaduragbemi1@gmail.com
Blueye x3 Connection with ROS

To make this package work with the Blueye, you need to complete a couple of steps.

1. Install the pyenv as specified in the documentation for using the Blueye SDK at https://blueye-robotics.github.io/blueye.sdk/v2.1/quick_start/

2. Clone this repo into your ROS1 workspace

3. Clone the DVL repo into your ROS1 workspace. The repo for the DVL A50 compatible with Blueye is at https://github.com/waterlinked/dvl-a50-ros-driver. 

4. Build your workspace using catkin_make or catkin build depending on your workspace command. This is to allow the custom messages to work.

5. Ensure that the IP address in the launch file works by just putting on a browser, if a GUI pops up, then it works.

6. Add the following lines to your bashrc file to activate the sdk env 

		# Initialize pyenv
		export PATH="$HOME/.pyenv/bin:$PATH"
		if command -v pyenv 1>/dev/null 2>&1; then
		  eval "$(pyenv init --path)"
		  eval "$(pyenv init -)"
		fi


6. Once connected to the Blueye through the surface unit, you can launch the whole system by running ./BlueyeController.sh on the terminal. It launches all the nodes and a GUI that has the control panel shows up. You can open the file on a texteditor and confirm that you source the correct ROS version and ROS workspace

7. You can create a desktop app by creating a file_name.desktop file and adding this in the file, give the path to the BlueyeController.sh in Exec

		[Desktop Entry]
		Version=1.0
		Name=Blueye Interface
		Comment=Start ROS launch files and node
		Exec=<--- path to "BlueyeController.sh" file ----->
		Icon=utilities-terminal
		Terminal=true
		Type=Application
		
		
8. Right click on the desktop app and Allow Launching on the desktop app created.
