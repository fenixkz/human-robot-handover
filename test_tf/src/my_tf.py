#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from tf.transformations import *

rospy.init_node('tf_listener')
br = tf.TransformBroadcaster()
rate = rospy.Rate(50.0)
while not rospy.is_shutdown():
    trans = [0.3, -0.5, 1.0]
    rot = [-0.9271151661621931, 0.025014685556883264, -0.07306975797285317, 0.3667322519881886]
    br.sendTransform(trans,rot,rospy.Time.now(), "camera_frame", "panda_link0")
