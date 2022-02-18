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
#include <iwh_arm/Vclassify.h>
#include <std_msgs/UInt8.h>

iwh_arm::GuiMsg guimsg;
std_msgs::UInt8 v_pick_topic;
iwh_arm::status status;
iwh_arm::Vclassify vclassify;

int main(int argc, char** argv)
{   
    int a,b=0;
    ros::init(argc, argv, "gui_2");
    ros::NodeHandle nh;
/*
	//Vison 2 한테 받는 data
	ros::Subscriber sub1 = nh.subscribe("V_Classify_topic", 1, iwh_vision2_topic_callback);
	//중앙roscore한테 받는 data
	ros::Subscriber sub2 = nh.subscribe("StartStop_Topic", 1, iwh_StartStop_callback);
*/
    ros::Publisher     pub = nh.advertise<iwh_arm::GuiMsg>("StartStop_Topic", 1);
    
    ros::Publisher     pub2 = nh.advertise<iwh_arm::Vclassify>("V_Classify_topic", 1);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        std::cout << "stop_pub : push the 1\n";
        
        std::cin >> a;
        guimsg.button = a;
        pub.publish(guimsg);

        // std::cin >> b;
        // vclassify.city = b;
        // pub2.publish(vclassify);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}