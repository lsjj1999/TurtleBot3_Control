#!/usr/bin/env python

import numpy as np
import rospy
import matplotlib.pyplot as plt
from Ellipse import Ellipse
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import dist

from pid import PID
from math import sqrt
import time

import concurrent.futures
from drivers_ppc import PurePursuitDriver

pid_distance = PID(1.0, 0.001, 0.01, setpoint=0)
pid_distance.output_limits = (-1, 1)

x = 0.0
y = 0.0
theta = 0.0
x_vel = 0.0
drivers = [PurePursuitDriver()]
es = Ellipse([[0,-3],[0,0],[1,-3]], 0, 0, 360)
# es = Ellipse([[0,-3],[0,0],[1,-3]], 0, 0, 360)
# es = Ellipse([[0,-2],[0,0],[2,-4]], 0, 0, 360)
goalx, goaly = es.ellipse(0.1)
ref = []
for a, b in zip(goalx[:], goaly[:]):
    ref.append([a,b])
ref = np.array(ref)


def newOdom(msg):
    global x
    global y
    global theta
    global x_vel

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    x_vel = msg.twist.twist.linear.x

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("pi_controller")
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)


# Reference path
# plt.plot(goalx, goaly)
# plt.title('Reference Path')
# plt.grid(True)
# plt.show()


x_error=[]
y_error=[]
x_path=[]
y_path=[]
t=[]
vel=[]
flag = False
l_idx = 0
lap = 0

while not flag:
    speed = Twist()
    actions = []
    futures = []
    _l_idx = l_idx
    with concurrent.futures.ThreadPoolExecutor() as executor:
        for i, driver in enumerate(drivers):
            futures.append(executor.submit(driver.pure_pursuit_control, x, y, theta, ref))
    for future in futures:
        velocity, steer, l_idx = future.result()
        actions.append([steer, velocity])
    actions = np.array(actions)
    speed.angular.z = steer
    print(steer)

    distance_to_goal = dist(ref[l_idx], [x,y])
    speed.linear.x = pid_distance(-distance_to_goal)
    pub.publish(speed)

    print(_l_idx, l_idx, lap, x_vel)
    if _l_idx - l_idx > 61:
        lap = lap + 1
        if lap == 1:
            flag = True

    t.append(time.clock_gettime(3))
    x_path.append(x)
    y_path.append(y)
    vel.append(x_vel)


    plt.clf()
    plt.plot(ref[:,0],ref[:,1],"*b", markersize = 1)
    plt.plot(ref[l_idx,0],ref[l_idx,1],"*g", markersize = 5)
    plt.plot(x, y,"*r", markersize = 5)
    plt.show(block=False)
    plt.grid(True)
    plt.pause(0.1)


# Path graph
print(t[len(t)-1], "seconds flow")
plt.clf()
plt.plot(ref[:,0],ref[:,1],"*r", markersize = 1)
plt.plot(x_path, y_path)
plt.title('Fusion path')
plt.grid(True)
plt.show()

# Velocity graph
plt.plot(t,vel)
plt.grid(True)
plt.show()