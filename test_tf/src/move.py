#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import tf
import std_msgs
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('franka_moveit',
                anonymous=True)
robot = moveit_commander.RobotCommander()
listener = tf.TransformListener()
pub = rospy.Publisher("/isMoving", std_msgs.msg.Float64, queue_size = 100)
pub_pregrasp = rospy.Publisher("/preGrasp", std_msgs.msg.Float64, queue_size = 100)
pub_ready = rospy.Publisher("/isReady", std_msgs.msg.Float64, queue_size=100)
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
grasp_ready = 0
grasp_not_ready = 0

rate = rospy.Rate(50.0)
pose_now = group.get_current_pose().pose
pose_goal = Pose()
msg = Float64()
while not rospy.is_shutdown():
    #print(trans,rot)
    data = None
    while data is None:
        try:
            data = rospy.wait_for_message("/isStopped", std_msgs.msg.Float64MultiArray,timeout=5)
        except:
            pass
    if (data.data[3] == 1):
        pose_goal.position.x = data.data[0] - 0.1
        pose_goal.position.y = data.data[1] - 0.15
        pose_goal.position.z = data.data[2] - 0.03
        pose_goal.orientation.x = 0.548192260384
        pose_goal.orientation.y = 0.462100514804
        pose_goal.orientation.z = 0.522167394535
        pose_goal.orientation.w = 0.461832839844
        #print(pose_goal)
        group.set_pose_target(pose_goal)
        group.plan()
    else:
        continue
    #plan = group.go(wait=True)
    #group.stop()
    #group.clear_pose_targets()
    #check if it came to the goal pose
    pose_now = group.get_current_pose().pose
    if (abs(pose_goal.position.x - pose_now.position.x) < 0.02 and abs(pose_goal.position.y - pose_now.position.y) < 0.02 and abs(pose_goal.position.z - pose_now.position.z) < 0.02):
        if(grasp_ready == 0):
            msg.data = 1
            pub_ready.publish(msg)
            grasp_ready = 1
            grasp_not_ready = 0

    else:
        if(grasp_not_ready == 0):
            msg.data = 0
            pub_ready.publish(msg)
            grasp_ready = 0
            grasp_not_ready = 1
#orientation: hand shake
#  x: 0.548192260384
#  y: 0.462100514804
#  z: 0.522167394535
#  w: 0.461832839844

#orientation: vertical
#  x: -0.697116325001
#  y: 0.0515220931635
#  z: -0.712740615531
#  w: 0.0580957683956
