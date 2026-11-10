# PX4 ROS 2 Offboard Control Node for Exploration and AruCo detection

This project is an implementation of an Offboard Control ROS 2 Node for the PX4 controller that autonomously pilots the drone along a pre-determined path read from a JSON file until the vision sensor on the vehicle identifies a certain ArUco marker (in the current implementation the marker with id=0), at which point the drone terminates the exploration phase and performs a controlled descent on the marker. 

# Dependencies
The project is built using the standard ROS 2 client, OpenCV, nhlomannjson and PX4 libraries for C++. In the future the behaviour tree library will be used as well, in fact is already present among the dependencies in CMakeLists.txt and in package.xml. 

# Use of OpenCV
