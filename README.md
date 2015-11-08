# virtual_ptu

Quick Start Guide for using the virtual_ptu package to control a dynamixel pan-tilt unit

1. Get the virtual_ptu source code.
	-Clone the virtual_ptu source code into your catkin workspace src folder.
		-From the catkin_ws src directory run: "git clone https://github.com/westpoint-robotics/virtual_ptu.git"
	-Directory structure should look like ~/catkin_ws/src/virtual_ptu
	
2. Get the dynamixel_motor source code.
	-Clone the dynamixel_motor source code into your catkin workspace src folder.
		-From the catkin_ws src directory run: "git clone https://github.com/arebgun/dynamixel_motor.git"
	-Directory structure should look like ~/catkin_ws/src/dynamixel_motor

3. Build the virtual_ptu and dynamixel_motor packages using the ros catkin build system.
	-From the catkin_ws directory run: "catkin_make"
	
4. Connect the Logitech Extreme 3D Pro Joystick to the computer via the USB.

5. Connect the pan-tilt unit to the computer via the USB2Dynamixel connector.
	-Also make sure that your dynamixels are powered via an external power source.
	
6. In a terminal run: "roscore"

7. Open up a new terminal and run: "roslaunch virtual_ptu ptu_joy.launch"
	-This launch file calls on two launch files
		-Dynamixel control manager: controller_manager.launch
		-Dynamixel control spawner: controller_spawner.launch
	-It also starts the joystick node "joy_node" 
	
8. Now you will be able to control the pan-tilt using the Logitech Joystick
	-The twisting motion on the joystick (Z-Axis) will provide the panning
	-The forward-back motion (Y-Axis) will provide the tilting

