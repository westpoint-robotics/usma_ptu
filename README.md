# usma_ptu
Pan Tilt Unit

Quick Start Guide for using the usma_ptu package to control a dynamixel pan-tilt unit.  These instructions can also be extended to control a dynamixel-based multi-link arm.  There are currently three interfaces: a touch screen, a joystick, and a myo band.

- Download the usma_ptu source code.
 - Clone the source code into your catkin workspace src folder.
 - `cd ~/catkin_ws/src`
 - `git clone https://github.com/westpoint-robotics/usma_ptu.git`
 - Directory structure should look like ~/catkin_ws/src/usma_ptu
 - `cd ..`
 - `catkin_make`

### For using a USB2Dynamixel adapter or equivalent:

- Install the [dynamixel_motor](http://wiki.ros.org/dynamixel_motor?distro=indigo) package.
- Connect a USB joystick. Esnure the ROS [joy](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) package is installed.
- Configuration and setup is based on the [dynamixel_controller tutorial](http://wiki.ros.org/dynamixel_controllers/Tutorials).
- Connect the pan-tilt unit to the computer via the USB2Dynamixel connector.
 - Pan motor ID is 1, tilit motor ID is 2.
 - To install the Dynamixel RoboPlus software, follow this [tutorial](http://support.robotis.com/en/software/roboplus_main.htm).
 - To configure the Dynamixel IDs, read this [tutorial](http://support.robotis.com/en/product/bioloid/beginnerkit/usefullinfo/dxl_configuration.htm#ID_Change)
 - Ensure that your dynamixels are powered via an external power source and connected to a hub.

![Alt Text](http://www.trossenrobotics.com/resize/shared/images/PImages/IL-6PHUB-c.jpg?bw=1000&bh=1000)
Picture courtesy of Trossen Robotics

- Open up a new terminal and run:
 - `roslaunch usma_ptu ptu_joy.launch`
 - This launch file calls on two launch files:
  - Dynamixel control manager: controller_manager.launch
  - Dynamixel control spawner: controller_spawner.launch
  - It also starts the joystick node.
- Now you will be able to control the pan-tilt using the joystick.
 - The left-right motion on the joystick (x-axis) will provide panning.
 - The forward-back motion (y-axis) will provide tilting.

### For using an Arbotix-M board:

