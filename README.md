# Visual-Inertial Navigation Filter for a UAV

This project is part of my MSc thesis at Cranfield University. The aim was to develop a [optical flow](https://en.wikipedia.org/wiki/Optical_flow)-based Visual-Inertial Navigation System (VINS) for a UAV in an indoor envrionment. In order to test the VINS I developed a simulation in ROS/Gazebo.

This repository contains the project files for the VINS and all the Gazebo files. For the UAV model I utilised the [Hector project](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor) from TU Darmstadt and as I had to modify a number of these files for one reason or antoher I've included them in the project. These are all the `hector_XXX` folders in the `src` folder.

After the paper based on this thesis has been published it will be linked [here]().


### Project Breakdown
All files for the VINS are in the aptly-named `vins` folder and the proejct can be divided into the following five sections:
###### Main Launch File
```
roslaunch hector_quadrotor_demo IRP_multicam.launch
```
This will run a launch file that will launch the simulator, RVIZ and all the Hector quadrotor stuff.

###### Navigation Filter
```
rosrun vins nav_filter
```
Will run the navigation filter, which will also output the estimate position to the tf tree under `fused_frame` and to the topic `/fused_state/odom`.

###### Optical Flow Algorithm
```
rosrun vins vel_estimator
```
This will run the Optical Flow algorithm, which will subscribe to the relative camera topics from Gazebo and publish the estimated velocity to `/raw_vel`.

###### Waypoint Navigator
```
rosrun vins trajectories
```
This will run the node that handles publishing the relevant waypoints to the topic that the position controller is subscribed to. The node will wait at the current waypoint until it is triggered to go to the next one. This is done by running the following command in a terminal: `rostopic pub /moveit std_msgs/String "y"`

###### Optical Flow Fun
```
rosrun vins of_test
```
This will run a simple script that will run an optical flow algorithm on the laptops webcam, outputting the result in a small window.

### Compatability
This project was developed with the following distributions/libraries:
- Ubuntu 16.04
- ROS Kinetic Kame
- Eigen/OpenCV/Boost
