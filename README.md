# TurtleBot3_Control
This ROS package is a practice for controlling the TurtleBot3 to move in an elliptic path.


## How to run the code
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

Setup the model name of turtlebot3. I used it for waffle_pi
> export TURTLEBOT3_MODEL=waffle_pi

Run the gazebo environment that will operate the turtlebot3.
> roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

Open another terminal and run rviz
> roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

Open another terminal and run controller based on your choice
> cd catkin_ws/src/TurtleBot3_Control/src

If you want to run PID and pure-pursuit control
> python3 controller_pidppc.py

If you want to run only PID control
> python3 controller_pid.py

If you want to run only pure-pursuit control
> python3 controller_ppc.py


## Reference
Ellipse path generation
https://github.com/sugbuv/path_planning.git

PID control of Turtlebot3 with ROS
https://github.com/aliy98/ROS-TurtleBot3-PID-Controller.git

Turtlebot3 gazebo simulation
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
