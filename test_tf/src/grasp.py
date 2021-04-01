#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
import sensor_msgs
from sensor_msgs.msg import JointState
import time
torque = [0.0]*7
torque_ = [0.0]*7
torque_filtered = [0.0]*7
SR = 1
ready = 0
def highPass(z, z_, out):
    T = 0.2;
    dt = 0.02;
    res = (out + z - z_)*T/(T + dt);
    return res

def torqueCb(data):
    global torque
    global torque_
    global torque_filtered
    global ready
    torque_ = torque[:]

    for i in range(7):
        torque[i] = data.effort[i]
        torque_filtered[i] = highPass(torque[i], torque_[i], torque_filtered[i])

def to_grasp():
    global torque_filtered
    sum_torques = abs(sum(torque_filtered))
    while sum_torques < 2.5:
        sum_torques = abs(sum(torque_filtered))
    return sum_torques > 2.5

def cb(data):
    global ready
    msg_to_grasp = String()
    msg_grasped = Float32()
    msg_to_grasp.data = 'envelop'
    msg_grasped.data = 1
    ready = data.data
    if (data.data == 1):
        msg = Float32()
        if (SR): #rigid
            msg.data = 0.7
        else:
            msg.data = 0.5
        pub_envelop_torque.publish(msg)
        if (to_grasp()):
            print("trying to grasp")
            pub_grasp.publish(msg_to_grasp)
            time.sleep(0.9)
            pub_grasped.publish(msg_grasped)


rospy.init_node('grasping')
sub = rospy.Subscriber('/isPreGrasp', Float32, cb)
pub_envelop_torque = rospy.Publisher('/allegroHand_0/envelop_torque', Float32, queue_size=1000)
pub_grasp = rospy.Publisher('/allegroHand_0/lib_cmd', String, queue_size = 1000)
pub_grasped = rospy.Publisher('/isGrasped', Float32, queue_size = 1000)
sub_torque = rospy.Subscriber("/franka_state_controller/joint_states", sensor_msgs.msg.JointState, torqueCb)

time.sleep(1)
rospy.spin()
