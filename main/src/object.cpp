#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>

int main(int argc, char** argv){

ros::init(argc, argv, "bins_tracker");
ros::NodeHandle node;
tf::TransformListener listener; //tf listener & transformations
tf::StampedTransform t_bin1;
std::vector<moveit_msgs::CollisionObject> collision_objects; //vector of objects
std::vector<std::string> object_ids; //vector of strings (names of objects)
object_ids.push_back("Bin1");
moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //planning interface
moveit_msgs::CollisionObject bin; //Create an object msg
shapes::Box box = shapes::Box(0.1,0.1,0.1);
shapes::Mesh* m = shapes::createMeshFromShape(box); //find mesh
shape_msgs::Mesh mesh;
shapes::ShapeMsg mesh_msg; //create a shape msg
shapes::constructMsgFromShape(m,mesh_msg);
ros::Duration(0.5).sleep();
mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

bin.header.frame_id = "panda_link0";
bin.meshes.resize(1); //scale
bin.meshes[0] = mesh; //mesh

bin.mesh_poses.resize(1); //vector resize
bin.mesh_poses[0].position.x = 0; //pose
bin.mesh_poses[0].position.y = 0;
bin.mesh_poses[0].position.z = 0;
bin.mesh_poses[0].orientation.w= 1.0;
bin.mesh_poses[0].orientation.x= 0;
bin.mesh_poses[0].orientation.y= 0;
bin.mesh_poses[0].orientation.z= 0;
bin.operation = bin.ADD; //add object to collitions

bin.id =object_ids[0]; //rename object
collision_objects.push_back(bin); //you can insert different objects using a vector of collition objects
planning_scene_interface.addCollisionObjects(collision_objects); //add objects to planning interface

bin.meshes.clear(); //Clear mesh required for MOVE operation (Only to avoid a warning)
bin.operation = bin.MOVE; //change operation to MOVE
while (node.ok()){
    //Listen to tfs
    try{
        listener.lookupTransform("panda_link0", "marker_frame", ros::Time(0), t_bin1);
    }

        catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
    }
    collision_objects.clear();
    bin.id = object_ids[0]; //bin 1
    bin.mesh_poses[0].position.x = t_bin1.getOrigin().x(); //set pose
    bin.mesh_poses[0].position.y = t_bin1.getOrigin().y();
    bin.mesh_poses[0].position.z = t_bin1.getOrigin().z();
    bin.mesh_poses[0].orientation.w= t_bin1.getRotation().w();
    bin.mesh_poses[0].orientation.x= t_bin1.getRotation().x();
    bin.mesh_poses[0].orientation.y= t_bin1.getRotation().y();
    bin.mesh_poses[0].orientation.z= t_bin1.getRotation().z();
    collision_objects.push_back(bin);
    planning_scene_interface.applyCollisionObjects(collision_objects); //apply changes to planning interface
    ros::Duration(0.5).sleep();
  }
planning_scene_interface.removeCollisionObjects(object_ids); //delete objects from planning interface*/

ros::shutdown(); //turn off
return 0;
}
