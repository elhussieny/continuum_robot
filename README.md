
https://www.youtube.com/watch?v=sLpNf3Wudt8

To run the demo visualization on youtube:

1- download the package from github inot your src folder of yor ROS workspace.

2- catkin_make your ROS workspace.

3- Launch the continuum robot visualizer

roslaunch continuum_robot visulize.launch

Notes:
- The number of segments for a continumm robot is defined as a parameter in the visulize.launch file.
- The continuum robot class depends on creating the URDF file at first with the specified number of sections which appears in the core_node.cpp file. If you change the construction of the robot again in the core file (number of sections, lengths, the radius of disks ..) you need to restart the launch file again because it looks for the URDF file at the beginning.


Cite as:

Seleem, Ibrahim A., Samy FM Assal, Hiroyuki Ishii, and Haitham El-Hussieny. "Guided pose planning and tracking for multi-section continuum robots considering robot dynamics." IEEE Access 7 (2019): 166690-166703.
