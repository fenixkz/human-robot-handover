#!/usr/bin/env python
import rospy
import sensor_msgs	
from sensor_msgs.msg import JointState
torque = [0.0]*7
torque_ = [0.0]*7
torque_filtered = [0.0]*7
def highPass(z, z_, out):
    T = 0.2;
    dt = 0.02;
    res = (out + z - z_)*T/(T + dt);
    return res

def torqueCb(data):
    global torque
    global torque_
    global torque_filtered
    torque_ = torque[:]


    for i in range(7):
        torque[i] = data.effort[i]
        torque_filtered[i] = highPass(torque[i], torque_[i], torque_filtered[i])
    print(sum(torque_filtered))

rospy.init_node("highpass")
sub = rospy.Subscriber("/franka_state_controller/joint_states", sensor_msgs.msg.JointState, torqueCb)
print(type(torque))
rospy.spin()
