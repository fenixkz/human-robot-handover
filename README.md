# Human-robot-handover
Human-Robot Handover 

Franka Emika + Allegro Hand packages


**main** package contains all necessary scripts

How to launch:
First launch Franka+Allegro
 roslaunch moveit_combined panda_control_moveit_rviz.launch
 
Then, launch realsense camera

  roslaunch realsense2_camera rs_camera.launch 
  
After that, launch aruco tracker

  roslaunch my_aruco_tracker usb_cam_stream_publisher_intel.launch 
  
And all the nodes

  roslaunch main nodes.launch
  
And, the main script

  rosrun main main.py
