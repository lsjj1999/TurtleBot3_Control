# TurtleBot3_Control
This ROS package is a practice for controlling the TurtleBot3 to move in an elliptic path. It has three options, PID control, pure-pursuit control, and using both of them. In the case of initial two, it was used only for steering, and for the last case, pure-pursuit control for steering and PID control for speed. While the code is in progress, the location of the turret can be checked through gazebo, rviz, and real-time plotted graphs, and when the code is complete, the entire path graph can be checked.


## Ready to run the code
The first step is to make a workspace directory 
> mkdir -p catkin_ws/src
> 
> cd catkin_ws/src
> 
> source /opt/ros/noetic/setup.bash
> 
> catkin_init_workspace

Then download dependency packages about running turtlebot3
> sudo apt install ros-noetic-turtlebot3-msgs
>
> sudo apt install ros-noetic-turtlebot3
> 
> pip3 install simple_pid
>
> git clone https://github.com/ROBOTIS-GIT/turtlebot3
>
> git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
>
> git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
> 
> git clone https://github.com/lsjj1999/TurtleBot3_Control.git

And build the code
> cd ..
> 
> catkin_make


## How to run package
Select the model name of turtlebot3. I used model for waffle_pi
> export TURTLEBOT3_MODEL=waffle_pi

Run the gazebo environment that will operate the turtlebot3.
> roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

Open another terminal and run rviz program
> roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

Open another terminal and run controller based on your choice
> cd catkin_ws/src/TurtleBot3_Control/src

If you want to run PID and pure-pursuit control
> python3 controller_pidppc.py

If you want to run only PID control
> python3 controller_pid.py

If you want to run only pure-pursuit control
> python3 controller_ppc.py

## Execution Results
If you run the background environment, you can see the gazebo and rviz program for the first.

![스크린샷, 2023-06-15 13-08-59+](https://github.com/lsjj1999/TurtleBot3_Control/assets/45039229/4fc14e3a-89b8-4391-a763-d04d6636ef25)


If you run the code, you can check the reference point and the location of the current turtle bot displayed in real time.


After one lap of turtlebot and it goes near to the initial point, the code terminated then and plot the graph of whole trajectory.

![ppc](https://github.com/lsjj1999/TurtleBot3_Control/assets/45039229/55dcae50-b6dc-4997-96b6-cf020bb403d0)


And in the case of controller_pidppc.py, it also plots velocity graph of turtlebot.

![pidppcvel](https://github.com/lsjj1999/TurtleBot3_Control/assets/45039229/a5ec7ec4-0035-4842-8aac-cd0715dce9fc)

## Reference
Ellipse path generation
https://github.com/sugbuv/path_planning.git

PID control of Turtlebot3 with ROS
https://github.com/aliy98/ROS-TurtleBot3-PID-Controller.git

Turtlebot3 gazebo simulation
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
