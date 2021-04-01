#include <stdio.h>
#include <ros/ros.h>
#include "bhand/BHand.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#define DOF_JOINTS 16
double current_position[DOF_JOINTS] = {0.0};
void jointCallback(const sensor_msgs::JointState::ConstPtr& msg){
      for (int i=0; i< DOF_JOINTS; i++){
        current_position[i] = msg->position[i]
      }
   }

int main(int argc, char **argv){
  ros::init(argc, argv, "bhand_test");
  ros::NodeHandle nh;
  pBHand = new BHand(eHandType_Right);
  ros::Subscriber sub = nh.subscribe("/allegroHand_0/joint_states", 1000, jointCallback);
  ros::Rate loop_rate(333);
  while (ros::ok()){
    ros::spinOnce();
  }
  return 0;
}
