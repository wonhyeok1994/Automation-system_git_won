#include <iostream>
#include <string>
#include <map>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>

void func_debug(const geometry_msgs::PoseStamped::ConstPtr& a)
{
    std::cout << "header:\n";
    std::cout << "  seq: " << a->header.seq << "\n";
    std::cout << "  time:\n";
    std::cout << "    sec: " << a->header.stamp.sec << "\n";
    std::cout << "    nsec: " << a->header.stamp.nsec << "\n";
    std::cout << "  frame_id: " << a->header.frame_id << "\n";
    std::cout << "pose:\n";
    std::cout << "  position:\n";
    std::cout << "    x: " << a->pose.position.x << "\n    y: " << a->pose.position.y << "\n    z: " << a->pose.position.z << "\n";
    std::cout << "  quaternion:\n";
    std::cout << "    x: " << a->pose.orientation.x << "\n    y: " << a->pose.orientation.y << "\n    z: " << a->pose.orientation.z << "\n    w: " << a->pose.orientation.w << "\n";
    std::cout << "--------------------\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_sub");

    ros::NodeHandle nh;
    ros::NodeHandle simple_nh("move_base_simple");

    // receive argv and set paramter ("start_goal" and "end_goal")
    //std::map<std::string, double> start_goal = {{"x",-3.0}, {"y",-1.95}, {"z",0.0}, {"roll",0.0}, {"pitch",0.0}, {"yaw",0.0}};
    //std::map<std::string, double> end_goal = {{"x",-2.0}, {"y",-0.9}, {"z",0.0}, {"roll",0.0}, {"pitch",0.0}, {"yaw",0.0}};
    
    ros::Subscriber sub_debug = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &func_debug);

    ros::spin();

    return 0;
}