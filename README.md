# Jasper Controller
An inverse dynamics controller for ABB IRB 120 robotic arm.

This project was written to familiarize with the ROS system and also to implement a
easy to use controller for the ABB IRB 120 arm. A joint space inverse dynamics
controller has been implemented to facilitate that.

## Known issues
1. "Could not load controller ... effort_controllers/JointEffortController' does not exist."
	Run `sudo apt install ros-<version>-ros-control ros-<version>-ros-controllers`
	Replace `<version>` with the version of ros currently being run.
