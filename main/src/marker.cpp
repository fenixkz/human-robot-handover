#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Header.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "marker");
    tf::TransformListener listener;
    ros::NodeHandle nh;
    listener.waitForTransform("/panda_link0", "/palm_link", ros::Time(), ros::Duration(1.0));
    ros::Rate loop_rate(100);
    ros::Publisher palm = nh.advertise<geometry_msgs::PoseStamped>("/link0_to_marker", 100);
    geometry_msgs::PoseStamped msg;
    std_msgs::Header h;
    while (ros::ok()){
      try
      {
        tf::StampedTransform echo_transform;
        listener.lookupTransform("/panda_link0", "/marker_frame", ros::Time(), echo_transform);
        tf::Vector3 v = echo_transform.getOrigin();
        h.stamp = ros::Time::now();
        msg.header = h;
        msg.pose.position.x = v.getX();
        msg.pose.position.y = v.getY();
        msg.pose.position.z = v.getZ();
        palm.publish(msg);
        //std::cout<<v.getX()<< std::endl;
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << listener.allFramesAsString()<<std::endl;
      }
      loop_rate.sleep();
    }
    return 0;
}
