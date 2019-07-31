#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "robot_tf_publisher");
    // Initialize node handler
    ros::NodeHandle nh;

    // Initialize rate
    ros::Rate r(100);

    // Initialize broadcaster
    tf::TransformBroadcaster broadcaster;

    while(nh.ok()){
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)), 
                ros::Time::now(), "base_link", "base_laser"));
        r.sleep();
    }
}