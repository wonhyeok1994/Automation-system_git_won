#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/status.h>

swiftpro::status status;

void subok_callback(const swiftpro::status& msg)
{
    printf("call_Back ok\n");
    std::cout << (int)(msg.status) << std::endl;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera1_node_a");
	ros::NodeHandle nh;

	ros::Subscriber  sub = nh.subscribe("V_Check_topic", 1, subok_callback);
	ros::Rate loop_rate(20);


	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}