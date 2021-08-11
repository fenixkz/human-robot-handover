#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
rospy.init_node("nodes")
rate = rospy.Rate(100)
aruco_pub = rospy.Publisher("/aruco_position",Vector3Stamped, queue_size = 100 )
allegro_pub = rospy.Publisher("/allegro_states", JointState, queue_size = 100 )
franka_pub = rospy.Publisher("/franka_states", JointState, queue_size = 100 )
aruco_msg = Vector3Stamped()
allegro_msg = JointState()

def aruco_cb(msg):
    global aruco_msg
    aruco_msg = msg

def allegro_cb(msg):
    global allegro_msg
    allegro_msg = msg

def franka_cb(msg):
    global rate
    global aruco_msg
    global allegro_msg
    global franka_pub
    global aruco_pub
    global allegro_pub
    aruco_pub.publish(aruco_msg)
    allegro_pub.publish(allegro_msg)
    franka_pub.publish(msg)
    rate.sleep()


aruco = rospy.Subscriber("/aruco_single/position", Vector3Stamped, aruco_cb)
allegro = rospy.Subscriber("/allegroHand_0/joint_states", JointState, allegro_cb)
franka = rospy.Subscriber("/franka_state_controller/joint_states", JointState, franka_cb)



rospy.spin()
