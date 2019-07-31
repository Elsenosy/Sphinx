#include<ros/ros.h>
#include<tf/transform_broadcaster.h>

int main(int argc, char** argv){

    // initialize ros node
    ros::init(argc, argv, "sphinx_tf_publisher");

    // Initialize node handle 
    ros::NodeHandle nh;

    // Initialize rate
    ros::Rate r(1);

   
    // Intialize broadcaster
    tf::TransformBroadcaster broadcaster;

  
    while(nh.ok()){
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)), ros::Time::now(), "base_link", "base_laser"
            )
        );
        r.sleep();
    }

    return 0;
}