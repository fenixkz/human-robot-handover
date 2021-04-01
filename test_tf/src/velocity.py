#!/usr/bin/env python
import rospy
import numpy as np
import tf
import std_msgs
import math
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3Stamped

def calc_dx(a,b):
    return math.sqrt(abs(a**2 - b**2))

def list_append(l, val):
    if (len(l) < 30):
        l.append(val)
    else:
        l.pop(0)
        l.append(val)

rospy.init_node("velocity_calculation")

old_val = list()
listener = tf.TransformListener()
pub = rospy.Publisher("/isStopped", std_msgs.msg.Float64MultiArray, queue_size = 10)

rate = rospy.Rate(50)
isStopped = 0
isMoving = 0
msg = Float64MultiArray()
position = None
while position is None:
    try:
        position = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout = 100)
    except:
        continue
print("Found a marker")
i = 0
pregrasp = 0
while not rospy.is_shutdown():
    dx = 0
    dy = 0
    dz = 0

    position = None
    while position is None:
        try:
            position = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout = 100)
        except:
            continue
    list_append(old_val, position.vector)
    if (len(old_val) == 30):
        dx = calc_dx(old_val[29].x, old_val[0].x)
        dy = calc_dx(old_val[29].y, old_val[0].y)
        dz = calc_dx(old_val[29].z, old_val[0].z)
    if (dx == 0):
        continue
    if (dx > 0.1 or dy > 0.1 or dz > 0.2): #Case when the marker moves
      if (isMoving == 0):
          print("------------")
          print("Still moving")
          print("------------")
          isStopped = 0
          isMoving = 1
          msg.data = [0,0,0,0]
          pub.publish(msg)
    else:
      if (dx < 0.1 and dy < 0.1 and dz < 0.2):
          k = 0
          for i in range(15):
              pos = None
              while pos is None:
                  pos = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout = 100)
              dx = calc_dx(pos.vector.x, old_val[29].x)
              dy = calc_dx(pos.vector.y, old_val[29].y)
              dz = calc_dx(pos.vector.z, old_val[29].z)
              if (dx > 0.1 or dy > 0.1 or dz > 0.2):
                  break
              k+=1
          if(k == 15):
              if (isStopped == 0):
                  print("+++++++")
                  print("Stopped")
                  print("+++++++")
                  isStopped = 1
                  isMoving = 0
                  trans = None
                  while trans is None:
                      try:
                          (trans,rot) = listener.lookupTransform("/panda_link0", "/marker_frame", rospy.Time(0))
                      except (tf.LookupException, tf.ConnectivityException):
                          continue
                  msg.data = [trans[0], trans[1], trans[2], 1]
                  pub.publish(msg)
    rate.sleep()
