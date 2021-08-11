#!/usr/bin/env python
import rospy
import numpy as np
import tf
import std_msgs
import math
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
values = []
isReady = 0
isPreGrasp = 0
def calc_dx(a,b):
    return math.sqrt(abs(a**2 - b**2))

def list_append(l, val):
    if (len(l) < 50):
        l.append(val)
    else:
        l.pop(0)
        l.append(val)

msg = Float32()

def isPG(data):
    global isPreGrasp
    isPreGrasp = data.data
    print("isPreGrasp "+str(isPreGrasp))

def isRcb(data):
    global isReady
    isReady = data.data
    print("isReady "+str(isReady))

def callback(data):
    global msg
    global isReady

    list_append(values, [data.vector.x, data.vector.y, data.vector.z, rospy.Time.now().to_nsec()])
    dx = 0
    dy = 0
    dz = 0
    dt = 1
    if (len(values) > 20):
        dx = calc_dx(values[5][0], values[0][0])
        dy = calc_dx(values[5][1], values[0][1])
        dz = calc_dx(values[5][2], values[0][2])
        dt = (values[5][3] - values[0][3])*1e-9
    vel_x = dx/dt
    vel_y = dy/dt
    vel_z = dz/dt

    if (dx < 0.1 and dy < 0.1 and dz < 0.2):
        msg.data = 0
    else:
        msg.data = 1
    if (isReady):
        pub.publish(msg)



rospy.init_node("position_calculation")
sub = rospy.Subscriber('/aruco_single/position',Vector3Stamped, callback)
sub_ready = rospy.Subscriber('/isReady', Float32, isRcb)
sub_pre_grasp = rospy.Subscriber('/isPreGrasp', Float32, isPG)
pub = rospy.Publisher('/isMoving', Float32, queue_size = 1000)
data = None

rospy.spin()
