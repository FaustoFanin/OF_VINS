# AVDC_IRP :tada:

This project utilises TU Darmstadt's Hector Quadrotor project, found [here](https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor).

However, this project includes a complete copy of the Hector files are some core files were modified for one reason or another.

### Software
This project was developed with the following distributions/libraries:
- Ubuntu 16.04
- ROS Kinetic Kame

###### Misc.
Furthermore, the following commands had to be run to fix some issues to get the Hector project to run:
```
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-unique-identifier
sudo apt-get install ros-kinetic-geographic-info
sudo apt-get install ros-kinetic-laser-geometry
sudo apt-get install ros-kinetic-tf-conversions
sudo apt-get install ros-kinetic-tf2-geometry-msgs
sudo apt-get install ros-kinetic-joy
```


### Project Breakdown
Lastly, the following commands will launch their respective nodes:
###### Main Launch File
```
roslaunch hector_quadrotor_demo IRP_multicam.launch
```
This will run a launch file that will launch the simulator, RVIZ and all the Hector quadrotor stuff.

###### Navigation Filter
```
rosrun cpp_test nav_filter
```
Will run the navigation filter, which will also output the estimate position to the tf tree under `fused_frame` and to the topic `/fused_state/odom`.

###### Optical Flow
```
rosrun cpp_test vel_estimator
```
This will run the Optical Flow algorithm, which will subscribe to the relative camera topics from Gazebo and publish the estimated velocity to `/raw_vel`. **Currently broken, needs to be fixed.**

###### Waypoint Navigator
```
rosrun cpp_test trajectories
```
This will run the node that handles publishing the relevant waypoints to the topic that the position controller is subscribed to. The node will wait at the current waypoint until it is triggered to go to the next one. This is done by running the following command in a terminal: `rostopic pub /moveit std_msgs/String "y"`
