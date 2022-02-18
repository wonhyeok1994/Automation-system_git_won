#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from platform import machine
import rospy
from GUI_turtle.msg import GuiMsg

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #   node / publisher init

rospy.init_node('gui_test_topic')

pub_machine = [0 for i in range(6)]

for i in range(6) :     # camera1 , arm1 , turtle , camera2 , arm2 , empty
    pub_machine[i] = rospy.Publisher("gui_topic", GuiMsg, queue_size=1)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #   rate init

rate_short = rospy.Rate(1)
rate = rospy.Rate(0.5)
rate_long = rospy.Rate(0.01)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #   machine 상태 확인용 GuiMsg 형 변수

global machine_msg  ;   machine_msg = [0 for i in range(6)]
for i in range(6) :
    machine_msg[i] = GuiMsg()
    machine_msg[i].button = 0
    machine_msg[i].machine = i
    machine_msg[i].m_state = 0
    machine_msg[i].t_course= 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #   function


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #   subscriber

msg = GuiMsg()

def copy_msg(msg) :
    machine_msg[msg.machine] = msg

def sub_start(msg) :
    print(" ----- sub_start(msg) ----- ")
    return

def sub_stop(msg) : 
    print(" ----- sub_stop(msg) / start ----- ")
    # copy_msg(msg) 빈 메시지가 들어오는거라 X
    
    # 전체 m_state, t_course 변경
    for i in range (5) :
        machine_msg[i].m_state = 3      # 3 : gray , off 로 하려면 0 으로 !
        machine_msg[i].t_course = 0
    
    # 변경한 Msg -> publish
    pub_machine[5].publish(machine_msg[5])      # 첫번째가 제대로 안받아져서..
    for i in range(5) :
        pub_machine[i].publish(machine_msg[i])
        rate_short.sleep()
    print(" ----- sub_stop(msg) / end ----- ")
    return

def sub_button(msg) : 
    if msg.button == 0 :
        sub_stop(msg)
    elif msg.button == 1 :
        sub_start(msg)
    return

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #   publisher

def pub_state_0 () :
    # machine_msg[0].m_state = 0
    # machine_msg[1].m_state = 0
    # machine_msg[2].m_state = 0
    # machine_msg[3].m_state = 0
    # machine_msg[4].m_state = 0
    return

def pub_state_1 () :
    machine_msg[0].m_state = 1
    machine_msg[1].m_state = 1
    #machine_msg[2].m_state = 1
    machine_msg[3].m_state = 1
    machine_msg[4].m_state = 1
    return

def pub_state_2 () :
    #machine_msg[0].m_state = 2
    #machine_msg[1].m_state = 2
    machine_msg[2].m_state = 2
    #machine_msg[3].m_state = 2
    #machine_msg[4].m_state = 2
    return

def pub_state_3 () :
    # machine_msg[0].m_state = 3
    # machine_msg[1].m_state = 3
    # machine_msg[2].m_state = 3
    # machine_msg[3].m_state = 3
    # machine_msg[4].m_state = 3
    return

# subscriber


#while not rospy.is_shutdown() :
while True :

    sub_button = rospy.Subscriber('StartStop_Topic', GuiMsg, sub_button, queue_size=1)
    
    msg = GuiMsg()

    pub_state_1()
    pub_state_2()
    machine_msg[2].t_course = 2

    # 변경한 Msg -> publish
    pub_machine[5].publish(machine_msg[5])      # 첫번째가 제대로 안받아져서..
    print(" ---------- publish [5] ---------- ")
    for i in range(6) :
        rate_short.sleep()
        pub_machine[i].publish(machine_msg[i])
        print(" ---------- publish [" + str(i) + "] ---------- ")

    #rate_long.sleep()
    rate.sleep()


# rospy.spin()





