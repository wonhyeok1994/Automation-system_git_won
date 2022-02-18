#include <iostream>
#include <sstream>
#include <typeinfo>
#include <cstring>
#include <stdlib.h>
#include <iwh_arm/SwiftproState.h>

#include <string>
#include <ros/ros.h>
#include <iwh_arm/position.h>
#include <iwh_arm/status.h>
#include <iwh_arm/GuiMsg.h>
#include <iwh_arm/Vclassify.h>
#include <std_msgs/UInt8.h>

iwh_arm::position pos;
iwh_arm::SwiftproState pos_2;
iwh_arm::status status;
iwh_arm::GuiMsg GuiMsg;
iwh_arm::Vclassify Vclassify;
std_msgs::UInt8 uint8;

// #define M_apr_pos 0
// #define M_place_pos 0
// #define P_apr_pos 0
// #define P_place_pos 0 
// #define G_apr_pos 0
// #define G_place_pos 0
#define j1_static 0
#define iwh_machine_arm_1 1
#define iwh_machine_amr_2 4
#define iwh_state_on 1
#define iwh_state_moving 2

int place_regi = 0;
int pick_regi = 0;
int StartStop_button = 0;
int hold_100_count = 0;

int vision_ok = 0;
int turtle_ok = 0; 

//int tmp_manual = 1; 	// manual mode ON
int tmp_manual = 0; 	// manual mode OFF

void iwh_StartStop_callback(const iwh_arm::GuiMsg& msg)
{
	StartStop_button = msg.button;
	std::cout<<"StartStop_button data :  "<<StartStop_button<<std::endl;
	if(StartStop_button == 0)
	{
		vision_ok = 0;
		turtle_ok = 0;
		place_regi = 999;
		pick_regi = 999;

		GuiMsg.machine = iwh_machine_arm_1;      //arm 1
		GuiMsg.m_state = 3;      //stop
	}else if( StartStop_button == 1)
	{
		place_regi = 0;
		pick_regi = 0;

		GuiMsg.machine = iwh_machine_arm_1;      //arm 1
		GuiMsg.m_state = 1;      //on

		//프로그램 재시작시 HOME위치로 이동
		ros::NodeHandle nh;
		ros::Publisher 	 pub = nh.advertise<iwh_arm::position>("iwh_ctl_topic_1", 1);
		ros::Publisher 	 pub_2 = nh.advertise<iwh_arm::SwiftproState>("iwh_state_topic_1", 1);
		pos.x=j1_static; pos.y=125; pos.z=45; pos_2.pump=0; //wait pos
		// iwh_move_rate.sleep();
		pub.publish(pos);	
		pub_2.publish(pos_2);
	}
}

/*
< status >				카메라 1 -> ARM 1
uint8 status

< Vclassify.msg >		카메라 2 -> ARM 2
float64 x
float64 y
float64 z
uint8 city
	//메뉴얼모드 : -1
	//판교,구리,서울 : 1,2,3


< GuiMsg.msg >			코어 -> ARM
time stamp				ARM -> 코어
uint8 button
uint8 machine
uint8 m_state
uint8 t_course

< uint8.msg >			터틀 -> 암
uint8 data
*/
void iwh_vision1_topic_callback(const iwh_arm::status& msg)
{
	vision_ok = msg.status;
	std::cout<<"Vision data :  "<<vision_ok<<std::endl;
}

void iwh_turtle_topic_callback(const std_msgs::UInt8& msg)
{
	turtle_ok = msg.data;
	std::cout<<"turtle data :  "<<turtle_ok<<std::endl;
}


int main(int argc, char** argv)
{	
	//웅크린자세 setting
	pos.x= j1_static; pos.y=125; pos.z=45; pos_2.pump=0;
	
	
	char a[15];
	printf("IWH_pose_node.cpp Main\n");
	ros::init(argc, argv, "IWH_pose_node_1");
	ros::NodeHandle nh;
/*
1 : V_Check_topic				Status.msg

2 : V_Classify_topic			Vclassify.msg
*/
	//Vision한테 받는 data
	ros::Subscriber sub1 = nh.subscribe("V_Check_topic", 1, iwh_vision1_topic_callback);
	//중앙roscore한테 받는 data
	ros::Subscriber sub2 = nh.subscribe("StartStop_Topic", 1, iwh_StartStop_callback);
	//터틀이 한테 받는 data
	ros::Subscriber sub3 = nh.subscribe("T_StartLoc_topic", 1, iwh_turtle_topic_callback);

	ros::Publisher 	 pub = nh.advertise<iwh_arm::position>("iwh_ctl_topic_1", 1);
	ros::Publisher 	 pub_2 = nh.advertise<iwh_arm::SwiftproState>("iwh_state_topic_1", 1);
	
	// To RosMaster
	ros::Publisher   pub_3 = nh.advertise<iwh_arm::GuiMsg>("Gui_Topic", 1);
	GuiMsg.machine = iwh_machine_arm_1;      //arm 1

	ros::Rate loop_rate(20);
	
	ros::Rate iwh_move_rate(0.25);
	
	//웅크린자세 setting
	pos.x= j1_static; pos.y=125; pos.z=45; pos_2.pump=0;
	iwh_move_rate.sleep();



	while (ros::ok())
	{
/* 
 {0.	125, 	45, 	0} 웅크린자세
 {0, 	90,		0,  	0} 대기자세
 {0, 	60, 	88,		0} 픽업, P_apr{0, 	80, 70, 0}
 {0, 	60 , 	38, 	0} 적재, P_apr{0, 	67, 27,	0}
*/		
	
	//std::cout<< " StartStop_button && city_dst  :  "<<StartStop_button<<" && "<<city_dst<<" :  " << "GuiMsg.m_state : "<<GuiMsg.m_state<<std::endl;
		if( vision_ok && turtle_ok && StartStop_button )
		{
			printf("Box 픽업 start , pick_regi : %d \n\n",pick_regi);

			if(pick_regi == 0)
			{
				pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=0; //wait pos
				iwh_move_rate.sleep();
				pick_regi++;
			}
			if(pick_regi == 2)
			{
				pos.x=j1_static; pos.y=80; pos.z=70; pos_2.pump=0; //apr pos
				iwh_move_rate.sleep();
				pick_regi++;
			}
			if(pick_regi == 4)
			{
				pos.x=j1_static; pos.y=60; pos.z=88; pos_2.pump=1; //pick pos
				iwh_move_rate.sleep();
				pick_regi++;
			}
			if(pick_regi == 6)
			{
				iwh_move_rate.sleep(); //wait time for suction 
				pos.x=j1_static; pos.y=80; pos.z=70; pos_2.pump=1; //apr pos
				iwh_move_rate.sleep();
				pick_regi++;
			}			
			if(pick_regi == 8)
			{
				pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=1; //wait pos
				iwh_move_rate.sleep();
			}
			if(pick_regi < 10 ) pick_regi++;

			if(pick_regi == 10)
			{
				switch ( (vision_ok && turtle_ok && StartStop_button) )
				{
					case 0:
						/* code */
						printf( "vision_ok && turtle_ok && StartStop_button error\n");
						break;
					case 1:
						/* code */
						printf("Box 적재 start , place_regi : %d ", place_regi);
						if(place_regi == 0)
						{
							pos.x=j1_static; pos.y=90; pos.z=0; pos_2.pump=1; //wait pos
							iwh_move_rate.sleep();
							if(place_regi == 0) {place_regi++; break;}
						}
						if(place_regi == 1)
						{
							pos.x=j1_static; pos.y=67; pos.z=27; pos_2.pump=1; //apr pos
							iwh_move_rate.sleep();
							if(place_regi == 1) {place_regi++; break;}
						}
						if(place_regi == 2)
						{
							pos.x=j1_static; pos.y=60; pos.z=38; pos_2.pump=1; //place pos
							iwh_move_rate.sleep();
							if(place_regi == 2) {place_regi++; break;}
						}
						if(place_regi == 3)
						{
							//iwh_move_rate.sleep(); //place time for suction 
							pos.x=j1_static; pos.y=67; pos.z=27; pos_2.pump=0; //apr pos
							iwh_move_rate.sleep();
							if(place_regi == 3) {place_regi++; break;}
						}			
						if(place_regi == 4)
						{
							pos.x=j1_static; pos.y=125; pos.z=45; pos_2.pump=0; //wait pos
							iwh_move_rate.sleep();
							if(place_regi == 4) { place_regi++; break; }
						}
						if(place_regi == 5)
						{
							place_regi = 0;
							pick_regi = 0;
							vision_ok=0; turtle_ok=0;
							break;
						}
						
						if(place_regi == 999)
						{
							printf( "StartStop_button stop button pressed \n");
							break;
						}
					default:
						printf("???????????? \n");
						break;
				}
			}else if (pick_regi == 999) {}
			
		}else if( tmp_manual == 1) //manual
		{
			// std::cout << "input the degree : j1,j2,j3,pump"<< std::endl;
			// std::cin >> a;
			// pos.x = atof(strtok(a, ","));    //첫번째 strtok 사용.
			// pos.y = atof(strtok(NULL, ","));     //자른 문자 다음부터 구분자 또 찾기
			// pos.z = atof(strtok(NULL, ","));
			// pos_2.pump = atoi(strtok(NULL, ","));
			// iwh_move_rate.sleep();
		}else if( (vision_ok && turtle_ok && StartStop_button) == 0 ){ 
			hold_100_count++;
			if(hold_100_count == 50)
			{
				printf("\n =====stop button and Vision doesn't work===== \n"); 
				hold_100_count = 0;
			}
			loop_rate.sleep();
		}
		
		if( (vision_ok && turtle_ok && StartStop_button) )
		{
			printf("published now!\n");
			pub.publish(pos);	
			pub_2.publish(pos_2);
		}else if( tmp_manual )
		{
			printf("published now! by manual\n");
			pub.publish(pos);	
			pub_2.publish(pos_2);
		}

		//( ( place_regi || pick_regi ) ==0 ) 움직임이 있는경우
		//start이고 움직이지는 경우
		if( (StartStop_button == 1) && ( ( place_regi || pick_regi ) ==0 ) ){
			GuiMsg.m_state = 1;
			pub_3.publish(GuiMsg);
		//움직이지는 경우
		}else if( place_regi || pick_regi ) { 
			GuiMsg.m_state = 2;
			pub_3.publish(GuiMsg);
		//stop이고 움직이지 않는 경우
		}else if ( (StartStop_button == 0) && ( ( place_regi || pick_regi ) ==0 ) ){
			GuiMsg.m_state = 3;
			pub_3.publish(GuiMsg);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


