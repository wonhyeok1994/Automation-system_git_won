#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

std_msgs::UInt8 uint8;

int main(int argc, char** argv)
{	
	int a;
	ros::init(argc, argv, "camera2_node_t");
	ros::NodeHandle nh;

	ros::Publisher 	 pub = nh.advertise<std_msgs::UInt8>("T_EndLoc_topic", 1);
	ros::Rate loop_rate(20);

    uint8.data = 1;
	printf("%d",ros::ok());

	while (ros::ok())
	{
		std::cout << "start_pub : push the 1\n";
		std::cin >> a;
		if (a == 1){
			pub.publish(uint8);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
