#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/*
* ROS tf broadcaster node **************************
 
 For publishing TF transform between truck 
 (base_link) and LiDAR (laser).

 By Jon Eivind Stranden @ NTNU 2019

****************************************************
*/

int main(int argc, char** argv){
  ros::init(argc, argv, "car_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 3.14, 1), tf::Vector3(0.28, 0.0, 0.27)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}