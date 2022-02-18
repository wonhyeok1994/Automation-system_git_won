#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/client/simple_action_client.h>// action Library Header File
#include <actionlib/client/terminal_state.h>      // Action Goal Status Header File
#include <actionlib_msgs/GoalStatus.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <GUI_turtle/GuiMsg.h>
//#include <rosgraph_msgs/Clock.h>

#define MACHINE 2
#define OFF 0
#define ON 1
#define MOVING 2
#define STOP 3
#define STARTTOEND 2
#define ENDTOSTART 1

static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac;
static ros::Publisher pub_goal;
static ros::Publisher pub_start;
static ros::Publisher pub_end;
static int seq;
static GUI_turtle::GuiMsg ping;
static ros::Rate *rate_ptr;
static ros::Rate *rate_spin;
static ros::Rate *rate_debug;

using GoalMap=std::map<std::string, std::map<std::string, double> >;
static GoalMap get_param_from_file(const std::string filename)
{
    std::map<std::string, std::map<std::string, double> > goal;
    std::ifstream fin;
    std::string ns;
    std::string name;
    double value;
    int i;

    fin.open(filename);

    if (fin.fail()) {
        std::cerr << "file not found: " << filename << std::endl;
        exit(100);
    }

    for (i=0; i<12 && !fin.eof(); i++) {
        fin >> ns >> name >> value;

        if (goal.find(ns) == goal.end()) {
            goal[ns] = std::map<std::string, double>();
        }

        goal[ns][name] = value;
    }

    fin.close();

    return goal;
}

static void set_goal_param(const std::string filename)
{
    ros::NodeHandle nh;
    GoalMap goal;
    std::string path;
    path = ros::package::getPath("turtle_node") + "/" + filename;
std::cout << path << "-------------------------------------------------\n";
    goal = get_param_from_file(path);

    for (auto g : goal) {
        nh.setParam(g.first, g.second);
    }
}

static void set_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q)
{
    double half_yaw = yaw * 0.5;  
    double half_pitch = pitch * 0.5;  
    double half_roll = roll * 0.5;

    double cos_yaw = cos(half_yaw);
    double sin_yaw = sin(half_yaw);
    double cos_pitch = cos(half_pitch);
    double sin_pitch = sin(half_pitch);
    double cos_roll = cos(half_roll);
    double sin_roll = sin(half_roll);

    q.x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    q.y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    q.z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    q.w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw; 
}

// transform type
//   from std::map to geometry_msgs::PoseStamped
static void set_pose_stamped(std::map<std::string, double>& goal_map, geometry_msgs::PoseStamped& pose_stamped)
{
    pose_stamped.header.seq = seq++;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.position.x = goal_map["x"];
    pose_stamped.pose.position.y = goal_map["y"];
    pose_stamped.pose.position.z = goal_map["z"];
    set_quaternion(goal_map["roll"], goal_map["pitch"], goal_map["yaw"], pose_stamped.pose.orientation);
}

// store pose of goal
//   from parameter to variable
static void get_goal(std::string name, geometry_msgs::PoseStamped& pose_stamped)
{
    ros::NodeHandle nh;
    std::map<std::string, double> goal;

    while(!nh.getParam(name, goal));
    
    std::cout << "[Parameter " << name << "]\n";
    for (auto& pair : goal)
        std::cout << pair.first << ": " << pair.second << '\n';

    set_pose_stamped(goal, pose_stamped);
}

// transform type
//   from PoseStamped to MoveBaseGoal
static void set_action_goal(geometry_msgs::PoseStamped& pose_stamped, move_base_msgs::MoveBaseGoal& goal)
{
    goal.target_pose = pose_stamped;
}

// get action_goal from parameter server
static void get_action_goal(std::string name, move_base_msgs::MoveBaseGoal& goal)
{
    geometry_msgs::PoseStamped pose_stamped;

    get_goal(name, pose_stamped);
    set_action_goal(pose_stamped, goal);
}

static void send_goal(std::string param, std::string topic, int t_course, ros::Publisher* pub)
{
    std_msgs::UInt8 temp_msg;
    move_base_msgs::MoveBaseGoal goal;
    actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ACTIVE, "");

    get_action_goal(param, goal); // goal: MoveBaseGoal

    std::string info_msg = "sending " + param;
    ROS_INFO("%s", info_msg.c_str());
    ac->sendGoal(goal);                 // send MoveBaseGoal to ActionServer
    
    ping.m_state = MOVING;
    ping.t_course = t_course;

    // debug-3
    std::cout << "debug-3\n";

    while (!((state = ac->getState()) == actionlib::SimpleClientGoalState::SUCCEEDED)){
        if (state == actionlib::SimpleClientGoalState::PREEMPTED) {
            ROS_INFO("stop sign: preempted");
            return;
        }
        //ROS_INFO("%s: %d", state.text_.c_str(), state.state_);
        //rate_ptr->sleep(); // 1Hz
        rate_spin->sleep(); // 20Hz
    }

    rate_debug->sleep();

    temp_msg.data = 1;
    
    info_msg = "publish " + topic;
    ROS_INFO("%s", info_msg.c_str());
    pub->publish(temp_msg);
    
    ping.m_state = ON;
    ping.t_course = 0;
}

static void send_start_goal(void)
{
    send_goal("start_goal", "/T_StartLoc_topic", ENDTOSTART, &pub_start);
}

static void send_end_goal(void)
{
    send_goal("end_goal", "/T_EndLoc_topic", STARTTOEND, &pub_end);
}


// callback function
//   that gets called on completion of place-action 
static void place_complete_cb(const std_msgs::UInt8::ConstPtr& msg)
{
    std_msgs::UInt8 temp_msg;
    move_base_msgs::MoveBaseGoal goal;

    if (msg->data == 10) {
        ROS_INFO("place_complete_cb");
        send_end_goal();
    }
}

// callback function
//   that gets called on copletion of pick-action
static void pick_complete_cb(const std_msgs::UInt8::ConstPtr& msg)
{
    if (msg->data == 5) {
        ROS_INFO("pick_complete_cb");
        send_start_goal();
    }
}

// callback function
//    that gets called when StartStop topic arrives
static void start_stop_cb(const GUI_turtle::GuiMsg::ConstPtr& msg)
{
    const int start = 1;
    const int stop = 0;

    if (msg->button == start) { //start
        ROS_INFO("start_stop_cb: start");
        send_start_goal();
    } else if (msg->button == stop) { //stop
        //stop
        ROS_INFO("start_stop_cb: stop");
        ping.button = 0;
        ping.machine = MACHINE;
        ping.m_state = STOP;
        ping.t_course = 0;
        ac->cancelGoal();
    }
}

// debug function
//   for subscribing
static void func_debug(const geometry_msgs::PoseStamped::ConstPtr& a)
{
    std::cout << "header:\n";
    std::cout << "  seq: " << a->header.seq << "\n";
    std::cout << "  time:\n";
    std::cout << "    sec: " << a->header.stamp.sec << "\n";
    std::cout << "    nsec: " << a->header.stamp.nsec << "\n";
    std::cout << "  frame_id: " << a->header.frame_id << "\n";
    std::cout << "pose:\n";
    std::cout << "  position:\n";
    std::cout << "    x: " << a->pose.position.x << "\n    y: " << a->pose.position.y << "\n    z: " << a->pose.position.z << "\n";
    std::cout << "  quaternion:\n";
    std::cout << "    x: " << a->pose.orientation.x << "\n    y: " << a->pose.orientation.y << "\n    z: " << a->pose.orientation.z << "\n    w: " << a->pose.orientation.w << "\n";
    std::cout << "--------------------\n";
}
static void cancel_cb(const std_msgs::UInt8::ConstPtr& msg)
{   
    //ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@");
    if (msg->data == 1) {
        //ROS_INFO("###############################cancel Goal");
        ac->cancelGoal();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_node");

    ros::NodeHandle nh;
    ros::NodeHandle simple_nh("move_base_simple");

    move_base_msgs::MoveBaseGoal goal;         // GoalID: O, PoseStamped: O


    // receive argv and set paramter ("start_goal" and "end_goal")
    /*
    std::map<std::string, double> start_goal = {{"x", 0.5}, {"y",0.5}, {"z",0.0}, {"roll",0.0}, {"pitch",0.0}, {"yaw",0.0}};
    std::map<std::string, double> end_goal = {{"x",-1.5}, {"y",-0.5}, {"z",0.0}, {"roll",0.0}, {"pitch",0.0}, {"yaw",0.0}};
    nh.setParam("start_goal", start_goal);
    nh.setParam("end_goal", end_goal);
    */
    //--------------------------------------
    
    //set param
    set_goal_param("param.txt");

    // debug-0
    std::cout << "debug-0\n";

    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);

    // debug-1
    std::cout << "debug-1\n";

    // 0. advertise
    pub_goal = simple_nh.advertise<geometry_msgs::PoseStamped>("goal", 100); // /move_base_simple/goal
    pub_start = nh.advertise<std_msgs::UInt8>("T_StartLoc_topic", 100);      // /T_
    pub_end = nh.advertise<std_msgs::UInt8>("T_EndLoc_topic", 100);
    ros::Publisher pub_ping = nh.advertise<GUI_turtle::GuiMsg>("Gui_Topic", 100);
    // 0.-------------------------------------

    // debug-1.2 : check /move_base_simple/goal by subscribe
    //ros::Subscriber sub_debug = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &func_debug);

    // 1. subscribe StartStop_topic => callback: msg.button.data==start: start_goal -> action -> publish T_StartLoc_topic
    //                                 callback: msg.button.data==stop: stop moving
    ros::Subscriber sub_gui = nh.subscribe("StartStop_Topic", 100, &start_stop_cb);
    // 1.-------------------------------------

    // 2. subscribe A_Place_topic => callback:  end_goal -> action -> publish T_EndLoc_topic
    ros::Subscriber sub_end = nh.subscribe("A_Place_topic", 100, &place_complete_cb);
    // 2.-------------------------------------
    
    // 3. subscribe A_Pick_topic => callback:  start_goal -> action -> publish T_StartLoc_topic
    ros::Subscriber sub_start = nh.subscribe("A_Pick_topic", 100, &pick_complete_cb);
    // 3.-------------------------------------

    // cancelGoal
    ros::Subscriber sub_cancel = nh.subscribe("cancel_topic", 100, &cancel_cb);


    rate_spin = new ros::Rate(20);
    rate_ptr = new ros::Rate(1);
    rate_debug = new ros::Rate(1.0/3.0);
    rate_ptr->sleep(); // 1Hz
    // waiting for advertising and subscribing

    ping.button = 0;
    ping.machine = MACHINE;
    ping.m_state = OFF;
    ping.t_course = 0;


    // debug-2
    std::cout << "debug-2\n";

    ros::AsyncSpinner spinner(4);

    spinner.start();
    ROS_INFO("out start()");
    while (ros::ok())
    {
        pub_ping.publish(ping);

        //ros::spinOnce();

        rate_spin->sleep(); //20Hz
    }

    delete ac;
    delete rate_ptr;
    delete rate_spin;

    return 0;
}

