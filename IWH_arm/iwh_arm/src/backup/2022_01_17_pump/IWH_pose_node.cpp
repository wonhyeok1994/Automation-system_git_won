#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cstring>
#include <stdlib.h>
#include <swiftpro/SwiftproState.h>

#include <string>
#include <ros/ros.h>
#include <swiftpro/position.h>

swiftpro::position pos;
swiftpro::SwiftproState pos_2;

/* 
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */

int main(int argc, char** argv)
{	
  char a[15];
  printf("IWH_pose_node.cpp START");
	ros::init(argc, argv, "IWH_pose_node");
	ros::NodeHandle nh;


	ros::Publisher 	 pub = nh.advertise<swiftpro::position>("iwh_ctl_topic", 1);
	
	ros::Publisher 	 pub_2 = nh.advertise<swiftpro::SwiftproState>("iwh_state_topic", 1);

	ros::Rate loop_rate(20);
	
	while (ros::ok())
	{
	  std::cout << "input the degree : j1,j2,j3,pump"<< std::endl;
	  std::cin >> a;
	  
	  pos.x = atof(strtok(a, ","));    //첫번째 strtok 사용.
    pos.y = atof(strtok(NULL, ","));     //자른 문자 다음부터 구분자 또 찾기
    pos.z = atof(strtok(NULL, ","));
    pos_2.pump = atoi(strtok(NULL, ","));
    
    printf("pos.x : %f , pos.y : %f , pos.z : %f pos.pump : %d \n",pos.x,pos.y,pos.z,pos_2.pump);
    printf(typeid(pos).name());

	  
		pub.publish(pos);
		
		pub_2.publish(pos_2);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}



