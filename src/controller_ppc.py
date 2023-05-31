#!/usr/bin/env python

import numpy as np
import rospy
import matplotlib.pyplot as plt
from Ellipse import Ellipse
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from simple_pid import PID
from math import sqrt
from math import dist
import time

import concurrent.futures
from drivers_ppc import PurePursuitDriver


x = 0.0
y = 0.0
theta = 0.0
drivers = [PurePursuitDriver()]
es = Ellipse([[0,-3],[0,0],[1,-3]], 0, 0, 360)
# es = Ellipse([[0,-2],[0,0],[2,-4]], 0, 0, 360)
goalx, goaly = es.ellipse(0.1)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

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

ref = []
for a, b in zip(goalx[:], goaly[:]):
    ref.append([a,b])
ref = np.array(ref)

x_path=[]
y_path=[]
t=[]
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
        velocity, steering, l_idx = future.result()
        actions.append([steering, velocity])
    actions = np.array(actions)

    # print(_l_idx, l_idx, lap)
    if _l_idx == 62 and l_idx == 0:
        lap = lap + 1
        if lap == 1:
            flag = True
        
    
    speed.linear.x = 0.5
    speed.angular.z = steering
    pub.publish(speed)

    t.append(time.clock_gettime(3))
    x_path.append(x)
    y_path.append(y)


    plt.clf()
    plt.plot(ref[:,0],ref[:,1],"*b", markersize = 1)
    plt.plot(ref[l_idx,0],ref[l_idx,1],"*g", markersize = 5)
    plt.plot(x, y,"*r", markersize = 5)
    plt.show(block=False)
    plt.grid(True)
    plt.pause(0.1)
    
# Path graph
print(t[len(t)-1],"seconds flow")
plt.clf()
plt.plot(ref[:,0],ref[:,1],"*r", markersize = 1)
plt.plot(x_path, y_path)
plt.title('Pure pursuit path')
plt.grid(True)
plt.show()