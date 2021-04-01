#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import tf
import time
import moveit_commander
import moveit_msgs.msg
import sys
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
import yaml
import os

rospy.init_node('fd')

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
listener = tf.TransformListener()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

print(group.get_current_joint_values())
joints_default = JointState()
joints_default.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4","panda_joint5","panda_joint6", "panda_joint7"]
joints_default.position = [0.009634743000070254, -0.9776191835303313, -0.004388718478239711, -2.6343597166347665, -0.008665640965931956, 1.6311944139492127, 0.7902435105759813]
group.set_joint_value_target(joints_default)
#joints_softbox = JointState()
#joints_softbox.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4","panda_joint5","panda_joint6", "panda_joint7"]
#joints_softbox.position = [-1.4521793538121814, 0.7201750791414064, 0.9429071854105966, -1.7844088949797206, 2.3276772192650617, 2.6460869647243963, 2.375144112906117]
#group.set_joint_value_target(joints_softbox)
plan = group.plan()
file_path = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan_from_soft_box_to_default.yaml')
with open(file_path, 'w') as file_save:
    yaml.dump(plan, file_save, default_flow_style=True)

#with open(file_path, 'r') as file_open:
#    loaded_plan = yaml.load(file_open)

#group.execute(loaded_plan, wait=True)

#tmp = [0.00793576145785368, -0.22525399267616097, -0.05878471213164347, -2.139820409145139, -0.03403160035560141, 1.8580988838913155, 0.7040672149804785]
#soft_box = [0.5238358019963697, 1.6386476169711588, -1.5262658885246385, -1.8549514301150307, -0.15053127115302617, 2.9023185792895094, 0.9427207298653327]
