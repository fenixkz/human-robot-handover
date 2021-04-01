#!/usr/bin/env python
import rospy
import sensor_msgs
from sensor_msgs.msg import JointState
rospy.init_node('hand')
angle_now = 0
def callback(data):
    global angle_now
    angle_now = data.position[1]

msg = JointState()
msg.name = ["joint_0", "joint_1", "joint_2", "joint_3",
"joint_4", "joint_5", "joint_6", "joint_7",
"joint_8", "joint_9", "joint_10", "joint_11",
"joint_12", "joint_13", "joint_14", "joint_15"]
msg.effort = [-0.02649219049841857,1,1,0.5,
0.0450241132971589,1,1,0.5,
0.07724058395235243,1,1,0.5,
0.17911132998501106,0.4,1,0.5]
msg_start = JointState()
msg_start.name = msg.name
msg_start.effort = [-0.02649219049841857, 0.5, 0, 0,
0, 0.5, 0, 0,
0, 0.5, 0, 0,
0, 0, 0.5, 0]
msg_envelop = JointState()
msg_envelop.name = msg.name
msg_envelop.effort = [-0.02649219049841857, 0.30000000447034836, 0.18000000268220903, 0.09000000134110452,
0.0450241132971589, 0.30000000447034836, 0.18000000268220903, 0.09000000134110452,
0.07724058395235243, 0.30000000447034836, 0.18000000268220903, 0.09000000134110452,
0.17911132998501106, -0.02711852302082533, 0.30000000447034836, 0.21000000312924386]

pub = rospy.Publisher("/allegroHand_0/torque_cmd", sensor_msgs.msg.JointState, queue_size=10)
sub = rospy.Subscriber("/allegroHand_0/joint_states", sensor_msgs.msg.JointState, callback)
rate = rospy.Rate(15.0)
while not rospy.is_shutdown():

    pub.publish(msg)

    rate.sleep()
rospy.spin()
