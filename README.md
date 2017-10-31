
https://www.youtube.com/watch?v=x0Cn3piElnU&feature=youtu.be

To run the demo visualization on youtube:

1- download the package from github.

2- catkin_make your ROS workspace.

3- $roscore

4- $rosrun continumm_robot core_node

5- roslaunch continuum_robot visulize.launch


Please note that, The continuum robot class depends on creating the URDF file at first with the specified number of sections which appears in the core_node.cpp file. If you change the construction of the robot again in the core file (number of sections, lengths, the radius of disks ..) you need to restart the launch file again because it looks for the URDF file at the beginning.


