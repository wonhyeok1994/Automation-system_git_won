#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <vision/GuiMsg.h>

vision::GuiMsg 		guimsg;

int main(int argc, char** argv)
{	
	int a;
	ros::init(argc, argv, "start_node");
	ros::NodeHandle nh;

    ros::Publisher 	 pub = nh.advertise<vision::GuiMsg>("StartStop_topic", 1);
	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		std::cout << "stop_pub : push the 1\n";
		std::cin >> a;
		guimsg.button = a;
		pub.publish(guimsg);
		// if (a == 1 || a == 0){
		// 	guimsg.button = a;
		// 	pub.publish(guimsg);
		// }
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}