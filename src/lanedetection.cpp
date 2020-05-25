#include "ros/ros.h"
#include "std_msgs/String.h"

// include for own ROS messages
#include "laneregression/point.h"
#include "laneregression/clusterData.h"

#include <sstream>

/**
 * This node publishes cluster points.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lanedetection_dummy");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<laneregression::clusterData>("lanedetection_cluster", 1000);

    ros::Rate loop_rate(10);

    // objects for publishing ROS topic
    laneregression::point ros_point;
    laneregression::clusterData ros_all_cluster;

    ros_point.x = 50;
    ros_point.y = 50;

    ros_all_cluster.points.push_back(ros_point);
    ros_all_cluster.size.push_back(1);

    std::cout << ros_all_cluster;

    chatter_pub.publish(ros_all_cluster);

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}