# Jasper Controller
An inverse dynamics controller for ABB IRB 120 robotic arm.

This project was written to familiarize with the ROS system and also to implement a
easy to use controller for the ABB IRB 120 arm. A joint space inverse dynamics
controller has been implemented to facilitate that.

## Demonstration
Trajectory following the natural human arm motion during feeding.

![Trajectory Following Demonstration](https://user-images.githubusercontent.com/18311491/89370547-dded2980-d695-11ea-8409-d6573de867e1.gif)

## Known issues
1. "Could not load controller ... effort_controllers/JointEffortController' does not exist."
	Run `sudo apt install ros-<version>-ros-control ros-<version>-ros-controllers`
	Replace `<version>` with the version of ros currently being run.
