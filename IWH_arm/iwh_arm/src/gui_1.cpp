/*
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iwh_arm/GuiMsg.h>
#include <std_msgs/UInt8.h>
*/

#include <string>
#include <ros/ros.h>
#include <iwh_arm/position.h>
#include <iwh_arm/status.h>
#include <iwh_arm/GuiMsg.h>
#include <iwh_arm/VClassify.h>
#include <std_msgs/UInt8.h>

iwh_arm::GuiMsg guimsg;
iwh_arm::status status;
std_msgs::UInt8 uint8;

int main(int argc, char** argv)
{   
    int a;
    ros::init(argc, argv, "gui_1");
    ros::NodeHandle nh;
/*
	//Vison 1 한테 받는 data
	ros::Subscriber sub1 = nh.subscribe("V_Check_topic", 1, iwh_vision2_topic_callback);
	//중앙roscore한테 받는 data
	ros::Subscriber sub2 = nh.subscribe("StartStop_Topic", 1, iwh_StartStop_callback);
	//터틀이 한테 받는 data
	ros::Subscriber sub3 = nh.subscribe("T_StartLoc_topic", 1, iwh_StartLoc_topic_callback);
*/
    // ros::Publisher     pub = nh.advertise<iwh_arm::GuiMsg>("StartStop_Topic", 1);
    // ros::Publisher     pub2 = nh.advertise<iwh_arm::status>("V_Check_topic", 1);
    ros::Publisher     pub3 = nh.advertise<std_msgs::UInt8>("T_StartLoc_topic", 1);
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        std::cout << "stop_pub : push the 1\n";
        std::cin >> a;
        // guimsg.button = a;
        // status.status = a;
        uint8.data = a;

        // pub.publish(guimsg);
        // pub2.publish(status);
        pub3.publish(uint8);


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}