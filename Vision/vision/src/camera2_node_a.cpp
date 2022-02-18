#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <swiftpro/Vclassify.h>

swiftpro::Vclassify v_classify;

void subok_callback(const swiftpro::Vclassify& msg)
{
    printf("call_Back ok\n");
    std::cout << msg.x << '\t' << msg.y << '\t' << (int)(msg.city) << std::endl;
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera2_node_a");
	ros::NodeHandle nh;

	ros::Subscriber  sub = nh.subscribe("V_Classify_topic", 1, subok_callback);
	ros::Rate loop_rate(20);


	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}