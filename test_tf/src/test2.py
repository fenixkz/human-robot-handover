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
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import copy
import os
import yaml
isStopped = 1
isMoving = 0
can_move = 0
pose_goal = Pose()
msg = Float32()
values = []
def list_append(l, val):
    if (len(l) < 10):
        l.append(val)
    else:
        l.pop(0)
        l.append(val)

def isGcb(data):
    isGrasped = data.data
    if (isGrasped):

        joints_default = JointState()
        joints_default.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4","panda_joint5","panda_joint6", "panda_joint7"]
        joints_default.position = [0.009634743000070254, -0.9776191835303313, -0.004388718478239711, -2.6343597166347665, -0.008665640965931956, 1.6311944139492127, 0.7902435105759813]
        group.set_joint_value_target(joints_default)
        plan = group.plan()
        executed_int = group.execute(plan,wait=True)


        #joints_int = JointState()
        #joints_int.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4","panda_joint5","panda_joint6", "panda_joint7"]
        #joints_int.position = [1.2519302268942996, -0.3806956553456547, 0.3053639585356792, -1.8630392212937115, 0.13064677491462867, 1.485090028958768, -0.8637298222858035]
        #group.set_joint_value_target(joints_int)
        #plan = group.plan()
        if (executed_int):
            executed = group.execute(loaded_plan0,wait=True)


        #joints_box = JointState()
        #joints_box.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4","panda_joint5","panda_joint6", "panda_joint7"]
        #joints_box.position = [0.9619653291261586, -1.6633890414798658, 1.4877964379841666, -2.3559260209934463, 0.4759878766883417, 2.8093783938825867, 0.15967366775239394]
        #group.set_joint_value_target(joints_box)
        #plan = group.plan()


        if (executed):
            isDone = group.execute(loaded_plan1, wait = True)
        group.stop()
        group.clear_pose_targets()

        if (isDone):
            msg = String()
            msg.data = "home"
            pub_release.publish(msg)
            time.sleep(0.5)


            pose_int = Pose()
            pose_int.position.x = -0.451021778924
            pose_int.position.y = 0.37893684051
            pose_int.position.z = 0.60118046923

            pose_int.orientation.x = -0.473732260251
            pose_int.orientation.y = 0.537220362139
            pose_int.orientation.z = 0.471387292531
            pose_int.orientation.w = 0.514554223129
            group.set_pose_target(pose_int)
            plan = group.plan()
            done2 = group.execute(loaded_plan2, wait=True)


            joints_default = JointState()
            joints_default.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4","panda_joint5","panda_joint6", "panda_joint7"]
            joints_default.position = [0.009634743000070254, -0.9776191835303313, -0.004388718478239711, -2.6343597166347665, -0.008665640965931956, 1.6311944139492127, 0.7902435105759813]
            group.set_joint_value_target(joints_default)

            plan = group.plan()
            if done2:
                done = group.execute(loaded_plan3, wait=True)

            group.stop()
            group.clear_pose_targets()

            if (done):
                msg_new_ready = Float32()
                msg_new_ready.data = 1.0
                pub.publish(msg_new_ready)

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

    if (can_move):
        pose_goal = group.get_current_pose().pose
        trans = None
        while trans is None:
            try:
                (trans,rot) = listener.lookupTransform("/panda_link0", "/marker_frame", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        pose_goal.position.x = trans[0] - 0.1
        pose_goal.position.y = trans[1] - 0.15
        pose_goal.position.z = trans[2]

        pose_goal.orientation.x = 0.548192260384
        pose_goal.orientation.y = 0.462100514804
        pose_goal.orientation.z = 0.522167394535
        pose_goal.orientation.w = 0.461832839844
        group.set_pose_target(pose_goal)
        msg = Float32()
        msg.data = 0
        pub.publish(msg)
        plan = group.plan()
        done = group.execute(plan, wait=True)
        time.sleep(0.5)
        group.stop()
        group.clear_pose_targets()

        if (done):
            can_move = 0
            print("Na meste")
            msg.data = 1.0
            pub_grasp.publish(msg)


rospy.init_node("wws")
pub = rospy.Publisher("/isReady", Float32, queue_size = 10)
pub_grasp = rospy.Publisher('/isPreGrasp', Float32, queue_size = 10)
sub_grasped = rospy.Subscriber("/isGrasped", Float32, isGcb)
pub_release = rospy.Publisher('/allegroHand_0/lib_cmd', String, queue_size = 1000)
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
listener = tf.TransformListener()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
group.set_planning_time(30)
sub = rospy.Subscriber("/isMoving", Float32, cb1)
pub_release.publish(String("ready"))
file_path_default_to_int = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan_to_intermediate_rigid.yaml')
with open(file_path_default_to_int, 'r') as file_open:
    loaded_plan0 = yaml.load(file_open)

file_path_int_to_box = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan_to_box_rigid.yaml')
with open(file_path_int_to_box, 'r') as file_open:
    loaded_plan1 = yaml.load(file_open)

file_path_box_to_int = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan_from_box_to_int.yaml')
with open(file_path_box_to_int, 'r') as file_open:
    loaded_plan2 = yaml.load(file_open)

file_path_box_to_default = os.path.join(os.path.expanduser('~'), 'saved_trajectories', 'plan_from_box_to_default.yaml')
with open(file_path_box_to_default, 'r') as file_open:
    loaded_plan3 = yaml.load(file_open)

msg.data = 1
print("Publishing")
numb_c = pub.get_num_connections()
while numb_c == 0:
    numb_c = pub.get_num_connections()
pub.publish(msg)
print("Published")
msg_allegro = String()
msg_allegro.data = "ready"
pub_release.publish(msg_allegro)
rospy.spin()
