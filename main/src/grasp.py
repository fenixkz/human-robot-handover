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
SR = None
ready = 0
isReleased = None
flag = 0
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
    while sum_torques < 1.5:
        sum_torques = abs(sum(torque_filtered))
    return sum_torques > 1.5

def class_cb(data):
    global SR
    SR = data.data
    if (SR):
        print("The object is Soft")
    else:
        print("The object is Rigid")

def release_cb(data):
    global isReleased
    isReleased = data.data
    #print(isReleased)
    if (isReleased):
        print("Released")

def cb(data):
    global ready
    global SR
    global isReleased
    global flag #temp
    msg_to_grasp = String()
    msg_grasped = Float32()
    msg_to_grasp.data = 'envelop'

    ready = data.data
    if (data.data == 1):
        msg = Float32()
        if (SR == 0): #rigid
            msg.data = 0.5
            msg_grasped.data = 0 #RIGID
        else: #soft
            msg.data = 0.3
            msg_grasped.data = 1 #SOFT
        pub_envelop_torque.publish(msg)
        time.sleep(0.5)
        if (to_grasp()):
            print("trying to grasp")
            pub_grasp.publish(msg_to_grasp)
            time.sleep(0.9)
            while not isReleased:
                pass
            if (isReleased):
                time.sleep(0.5)
                pub_grasped.publish(msg_grasped)


rospy.init_node('grasping')
sub = rospy.Subscriber('/isPreGrasp', Float32, cb)
pub_envelop_torque = rospy.Publisher('/allegroHand_0/envelop_torque', Float32, queue_size=1000)
pub_grasp = rospy.Publisher('/allegroHand_0/lib_cmd', String, queue_size = 1000)
pub_grasped = rospy.Publisher('/isGrasped', Float32, queue_size = 1000)
sub_class = rospy.Subscriber('/class', Float32, class_cb)
sub_released = rospy.Subscriber('/release', Float32, release_cb)
sub_torque = rospy.Subscriber("/franka_state_controller/joint_states", sensor_msgs.msg.JointState, torqueCb)
time.sleep(1)
rospy.spin()
