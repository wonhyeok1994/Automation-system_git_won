#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/status.h>
#include <vision/GuiMsg.h>

vision::GuiMsg GuiMsg;

int main(int argc, char** argv)
{	
	int a;
	ros::init(argc, argv, "start_node");
	ros::NodeHandle nh;

	ros::Publisher 	 pub = nh.advertise<vision::GuiMsg>("Start_topic", 1);
	ros::Rate loop_rate(20);

    GuiMsg.m_state = 1;

	while (ros::ok())
	{
		std::cout << "start_pub : push the 1\n";
		std::cin >> a;
		if (a == 1){
			pub.publish(GuiMsg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}