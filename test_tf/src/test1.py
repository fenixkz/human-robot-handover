#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import tf

isStopped = 1
isMoving = 0
can_move = 0
values = []
def list_append(l, val):
    if (len(l) < 15):
        l.append(val)
    else:
        l.pop(0)
        l.append(val)



def cb1(data):
    global isStopped
    global can_move
    global isMoving
    global pose_goal

    list_append(values, data.data)
    if (sum(values) == 0 and isStopped == 0):
        print("Still")
        can_move = 1
        isStopped = 1
        isMoving = 0
    elif (sum(values) != 0 and isMoving == 0):
        print("Moving")
        can_move = 0
        isMoving = 1
        isStopped = 0
rospy.init_node("wws")
pub = rospy.Publisher("/isReady", Float32, queue_size = 10)

sub = rospy.Subscriber("/isMoving", Float32, cb1)
msg = Float32()
msg.data = 1
numb_c = pub.get_num_connections()
print(numb_c)
while numb_c == 0:
    numb_c = pub.get_num_connections()

pub.publish(msg)
print("Published")
rospy.spin()
