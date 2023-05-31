# TurtleBot3_Control
This ROS package is a practice for controlling the TurtleBot3 to move in an elliptic path 

이 코드의 의의

$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

패키지 사용 방법(turtlebot 튜토리얼에 올라온 패키지부터 다운받기)
The first step is to make a workspace directory 
> mkdir -p catkin_ws/src 
> 
> cd catkin_ws/src
>
> source /opt/ros/noetic/setup.bash
>
> catkin_init_workspace
>
그리고 터틀봇 튜토리얼로 받는거 쓰고
git에서 받는거 쓰고
catkin_make 하면 실행준비완료

완전히 내가짠 코드는 아닌데 -> 레퍼 달기?

할거
pid 패키지 다운받는걸로 옮기기

export TURTLEBOT3_MODEL=waffle_pi

gazebo실행
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rviz실행
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
컨트롤러 실행
cd catkin_ws/src/TurtleBot3_Control/src
python3 controller_pidppc.py
or
python3 controller_pid.py
or
python3 controller_ppc.py


Reference
Ellipse path generation
https://github.com/sugbuv/path_planning.git

PID control of Turtlebot3 with ROS
https://github.com/aliy98/ROS-TurtleBot3-PID-Controller.git

Turtlebot3 gazebo simulation
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

