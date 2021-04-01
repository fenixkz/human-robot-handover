#!/usr/bin/env python
import rospy
import sensor_msgs
from sensor_msgs.msg import JointState
rospy.init_node('hand')
angle_now = 1
def callback(data):
    global angle_now
    angle_now = data.position[12]
msg = JointState()
msg.name = ["joint_0", "joint_1", "joint_2", "joint_3",
"joint_4", "joint_5", "joint_6", "joint_7",
"joint_8", "joint_9", "joint_10", "joint_11",
"joint_12", "joint_13", "joint_14", "joint_15"]
effort_ready = [-0.0035431431606411934, 0.07291218638420105, 0.027888912707567215, 0.006377029698342085,
0.002887047128751874, 0.06403619050979614, 0.016683725640177727, 0.0025450927205383778,
0.018420487642288208, 0.0615820474922657, 0.039010077714920044, 0.016066791489720345,
-0.06, -0.06, -0.08903010189533234, -0.03943688049912453]
effort_destroy = [0,0.5,0.3,0.3,
0,0.5,0.3,0.3,
0,0.5,0.3,0.3,
0,0,0,0]
effort_big_finger_away = [0,0,0,0,
0,0,0,0,
0,0,0,0,
-0.1,-0.1,0,0]
msg.effort = effort_destroy
pub = rospy.Publisher("/allegroHand_0/torque_cmd", sensor_msgs.msg.JointState, queue_size=10)
sub = rospy.Subscriber("/allegroHand_0/joint_states", sensor_msgs.msg.JointState, callback)
rate = rospy.Rate(15.0)
while not rospy.is_shutdown():
    if(angle_now > 0.3):
        msg.effort = effort_big_finger_away
    else:
        msg.effort = effort_destroy
    pub.publish(msg)
    rate.sleep()



#effort: [-0.0035431431606411934, 0.07291218638420105, 0.027888912707567215, 0.006377029698342085, 0.002887047128751874, 0.06403619050979614, 0.016683725640177727, 0.0025450927205383778, 0.018420487642288208, 0.0615820474922657, 0.039010077714920044, 0.016066791489720345, 0.02735593169927597, 0.1043860986828804, -0.08903010189533234, -0.03943688049912453]
