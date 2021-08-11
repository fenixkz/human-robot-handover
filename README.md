# Human-robot-handover
Human-Robot Handover 

Franka Emika + Allegro Hand packages


**main** package contains all necessary scripts

How to launch:
First launch Franka+Allegro
 _**roslaunch moveit_combined panda_control_moveit_rviz.launch**_
 
Then, launch realsense camera

  _**roslaunch realsense2_camera rs_camera.launch**_
  
After that, launch aruco tracker

  _**roslaunch my_aruco_tracker usb_cam_stream_publisher_intel.launch** _
  
And all the nodes

  _**roslaunch main nodes.launch**_
  
And, the main script

  _**rosrun main main.py**_
