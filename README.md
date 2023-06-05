## TurtleBot3_Control
This ROS package is a practice for controlling the TurtleBot3 to move in an elliptic path.


# How to install the dependency packages and run the code
The first step is to make a workspace directory 
> mkdir -p catkin_ws/src 
> 
> cd catkin_ws/src
>
> source /opt/ros/noetic/setup.bash
>
> catkin_init_workspace
>

Then download dependency packages about running turtlebot3
> sudo apt install ros-noetic-turtlebot3-msgs
> 
> sudo apt install ros-noetic-turtlebot3
>
> git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
>
> git clone https://github.com/lsjj1999/TurtleBot3_Control.git

And build the code
> cd ..
>
> catkin_make

If you don't have package of PID, ~~
할거
pid 패키지 다운받는걸로 옮기기


export TURTLEBOT3_MODEL=waffle_pi

run gazebo
> roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

run rviz
> roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

run controller based on your choice
> cd catkin_ws/src/TurtleBot3_Control/src
> python3 controller_pidppc.py
or
> python3 controller_pid.py
or
> python3 controller_ppc.py



# Reference
Ellipse path generation
https://github.com/sugbuv/path_planning.git

PID control of Turtlebot3 with ROS
https://github.com/aliy98/ROS-TurtleBot3-PID-Controller.git

Turtlebot3 gazebo simulation
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
