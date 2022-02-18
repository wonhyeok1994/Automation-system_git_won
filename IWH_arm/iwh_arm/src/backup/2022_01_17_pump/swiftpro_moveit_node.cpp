/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>   
 */
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <swiftpro/position.h>

#define MATH_PI 				3.141592653589793238463
#define MATH_TRANS  			57.2958    
#define MATH_L1 				106.6
#define MATH_L2 				13.2
#define MATH_LOWER_ARM 			142.07
#define MATH_UPPER_ARM 			158.81

//float motor_angle[3] = {90.0, 90.0, .0};
float motor_angle[3] = {90.0, 90.0, 0.0};
	
/* 
 * Description: forward kinematics of swift pro
 * Inputs: 		angle[3]			3 motor angles(degree)
 * Outputs:		position[3]			3 cartesian coordinates: x, y, z(mm)
 */
void swift_fk(float angle[3], float position[3])
{
	double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS) 
				   + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

	double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS) 
				  - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;
	
	position[0] = stretch * sin(angle[0] / MATH_TRANS);
	position[1] = -stretch * cos(angle[0] / MATH_TRANS);
	position[2] = height - 74.55;
}

void iwh_ctl_topic_callback(const swiftpro::position& msg)
{
  
  motor_angle[0] = msg.x;
  motor_angle[1] = msg.y;
	motor_angle[2] = msg.z;
}

int main(int argc, char **argv)
{
  printf("moveit_node.cpp START");
	float position[3];
	swiftpro::position pos;
	
	ros::init(argc, argv, "swiftpro_moveit_node");
	ros::NodeHandle n;
	
	ros::Subscriber sub1 = n.subscribe("iwh_ctl_topic", 1, iwh_ctl_topic_callback);
		
	ros::Publisher 	pub = n.advertise<swiftpro::position>("position_write_topic", 1);
	ros::Rate loop_rate(20);
	
	while (ros::ok())
	{
		swift_fk(motor_angle, position);
//swift_fk(motor_angle, position); 모토앵글에서 data받아서 로봇에 맞게 가공해서 postion으로 리턴 해줌
//위에 joint_Callback 함수가 call안되면 전역선언된 (60,60,60)data를 가공해서 pos.xyz에 넣어줌
		pos.x = position[0];
		pos.y = position[1];
		pos.z = position[2];
		pub.publish(pos);
		
    //HOME 포지션 발행 (60,60,60)
		ros::spinOnce();
		loop_rate.sleep();
	}		
	return 0;
}

