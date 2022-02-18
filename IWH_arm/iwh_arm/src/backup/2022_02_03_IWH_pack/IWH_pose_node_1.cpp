#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cstring>
#include <stdlib.h>
#include <swiftpro/SwiftproState.h>

#include <string>
#include <ros/ros.h>
#include <swiftpro/position.h>
#include <swiftpro/status.h>

swiftpro::position pos;
swiftpro::SwiftproState pos_2;
swiftpro::status status;

int city_dst = -1;
/* 
 {90.0, 125.0, 45.0} 웅크린자세
 {90, 90,  0,  0} 대기자세
 {90, 110, 45, 0} 문래	, M_apr{90,125,20,0}
 {90, 80 , 45, 0} 판교 	, P_apr{90,95,20,0}
 {90, 50 , 45, 0} 구리	, G_apr{90,65, 20,0}
 */
void iwh_vision2_topic_callback(const swiftpro::status& msg)
{
	city_dst = msg.status;
	std::cout<<"subscribed data :  "<<city_dst<<std::endl;
}


int main(int argc, char** argv)
{	
	int j1_static = 0;
	int place_regi = 0;
	int pick_regi = 0;

	int M_apr_pos ,M_place_pos = 0;
	int P_apr_pos ,P_place_pos = 0;
	int G_apr_pos ,G_place_pos = 0;

	char a[15];
	printf("IWH_pose_node.cpp Main\n");
	ros::init(argc, argv, "IWH_pose_node");
	ros::NodeHandle nh;

	ros::Subscriber sub1 = nh.subscribe("V_Check_topic", 1, iwh_vision2_topic_callback);

	ros::Publisher 	 pub = nh.advertise<swiftpro::position>("iwh_ctl_topic_1", 1);
	ros::Publisher 	 pub_2 = nh.advertise<swiftpro::SwiftproState>("iwh_state_topic_1", 1);

	ros::Rate loop_rate(20);
	
	ros::Rate iwh_move_rate(0.25);

	pos.x= j1_static; pos.y=125; pos.z=45; pos_2.pump=0;
	iwh_move_rate.sleep();
	while (ros::ok())
	{
/* 
 {90.0, 125.0, 45.0} 웅크린자세
 {90, 90,  0,  0} 대기자세
 {90, 80 , 45, 0} 픽업  , P_apr{90, 95, 20, 0}

 {90, 110, 45, 0} 문래	, M_apr{90, 125,20, 0}
 {90, 80 , 45, 0} 판교 	, P_apr{90, 95, 20, 0}
 {90, 50 , 45, 0} 구리	, G_apr{90, 65, 20, 0}
 */
	if( 1==city_dst )
	{
		printf("Box 픽업 start , pick_regi : %d \n\n",pick_regi);

		if(pick_regi == 0)
		{
			pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=0; //waite pos
			iwh_move_rate.sleep();
			pick_regi++;
		}
		if(pick_regi == 2)
		{
			pos.x=j1_static; pos.y=95; pos.z=20; pos_2.pump=0; //apr pos
			iwh_move_rate.sleep();
			pick_regi++;
		}
		if(pick_regi == 4)
		{
			pos.x=j1_static; pos.y=80; pos.z=45; pos_2.pump=1; //place pos
			iwh_move_rate.sleep();
			pick_regi++;
		}
		if(pick_regi == 6)
		{
			iwh_move_rate.sleep(); //waite time for suction 
			pos.x=j1_static; pos.y=95; pos.z=20; pos_2.pump=1; //apr pos
			iwh_move_rate.sleep();
			pick_regi++;
		}			
		if(pick_regi == 8)
		{
			pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=1; //waite pos
			iwh_move_rate.sleep();
		}
		if(pick_regi < 10 ) pick_regi++;

		if(pick_regi == 10)
		{
			switch (city_dst)
			{
			case 0:
				/* code */
				printf("box doesn't exit\n");
				city_dst = -1;
				break;
			case 1:
				/* code */
				printf("bos exits , place_regi : %d \n\n",place_regi);

				if(place_regi == 0)
				{
					pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=1; //waite pos
					iwh_move_rate.sleep();
					if(place_regi == 0) {place_regi++; break;}
				}
				if(place_regi == 1)
				{
					pos.x=j1_static; pos.y=125; pos.z=20; pos_2.pump=1; //apr pos
					iwh_move_rate.sleep();
					if(place_regi == 1) {place_regi++; break;}
				}
				if(place_regi == 2)
				{
					pos.x=j1_static; pos.y=110; pos.z=45; pos_2.pump=1; //place pos
					iwh_move_rate.sleep();
					if(place_regi == 2) {place_regi++; break;}
				}
				if(place_regi == 3)
				{
					pos.x=j1_static; pos.y=125; pos.z=20; pos_2.pump=0; //apr pos
					iwh_move_rate.sleep();
					if(place_regi == 3) {place_regi++; break;}
				}			
				if(place_regi == 4)
				{
					pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=0; //waite pos
					iwh_move_rate.sleep();
					if(place_regi == 4) 
					{
					place_regi++; 
					place_regi = 0;
					pick_regi = 0;
					city_dst = -1;
					break;
					}
				}
			default:
				break;
			}
		}
	}else
	{
		// std::cout << "input the degree : j1,j2,j3,pump"<< std::endl;
		// std::cin >> a;
		// pos.x = atof(strtok(a, ","));    //첫번째 strtok 사용.
		// pos.y = atof(strtok(NULL, ","));     //자른 문자 다음부터 구분자 또 찾기
		// pos.z = atof(strtok(NULL, ","));
		// pos_2.pump = atoi(strtok(NULL, ","));
		// //iwh_move_rate.sleep();
	}
	
	printf("pos.x : %f , pos.y : %f , pos.z : %f pos.pump : %d \n",pos.x,pos.y,pos.z,pos_2.pump);
	pub.publish(pos);	
	pub_2.publish(pos_2);
	ros::spinOnce();
	loop_rate.sleep();
	//printf("iwh_end\n");
	}
	return 0;
}



