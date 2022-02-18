#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from platform import machine
import rospy
import threading
import time     # for time.sleep()
import tkinter
import os	# terminal clear
import tkinter.font     # font 사용을 위함
from datetime import datetime   # 현재시간 출력을 위함
from GUI_turtle.msg import GuiMsg

# button 과 label 크기 지정
global toplabel_w, toplabel_h, button_w, button_h, name_w, name_h, sign_w, sign_h, course_w, course_h      
toplabel_w = 634   ; toplabel_h = 101   ;   button_w = 285  ;   button_h = 99
name_w = 160  ;   name_h = 55 ;   sign_w = 81 ;   sign_h = 55 ;   course_w = 365  ;   course_h = 55

# label 위치 값 x, y 
global top_x, top_y, top_pd
global btn_pd_w, btn_pd_h, btn_add, btn_x1, btn_y1, btn_x2, btn_y2, btn_add2
global label_pd_w, label_pd_h, label_name_add, label_sign_add, label_add1, label_add2
global label_x1, label_x2, label_x3, label_y1, label_y2, label_y3, label_y4, label_y5 

top_x = 18  ;   top_y = 20  ; top_pd = 17                                  # TOP x,y, padding
btn_pd_w = 41   ;   btn_pd_h = 25   ;   btn_add = button_w + btn_pd_w       # START/STOP button x,y, padding
btn_x1 = top_x  ; btn_y1 = top_y+toplabel_h+top_pd ;   btn_x2 = btn_x1+btn_add    
label_pd_w = 14   ;   label_pd_h = 4                                        # NAME, SIGN , COURSE x,y, padding
label_name_add = name_w+label_pd_w  ;  label_sign_add = sign_w+label_pd_w   
label_add1 = button_h+btn_pd_h  ;   label_add2 = name_h+label_pd_h          
label_x1 = btn_x1  ;   label_x2 = label_x1+label_name_add   ;   label_x3 = label_x2+label_sign_add
label_y1 = btn_y1+label_add1 ;   label_y2 = label_y1+label_add2 ;   label_y3 = label_y2+label_add2
label_y4 = label_y3+label_add2  ;   label_y5 = label_y4+label_add2 
btn_add2 = name_h+top_pd  ;   btn_y2 = label_y5+btn_add2

# machine 상태 확인용 GuiMsg 형 변수 _ 0~4 , 5
#global t_signal_check                               # signal check 할 thread
#global v_signal_check   ;   v_signal_check = 0      # signal check 할 variable
global machine_msg  ;   machine_msg = [0 for i in range(6)]
for i in range(6) :
    machine_msg[i] = GuiMsg()
    machine_msg[i].machine = i
    machine_msg[i].m_state = 0

# machine name 혹은 state 를 터미널에 출력할 때 사용
global name_machine     ;   name_machine = {0:"camera 1", 1:"arm 1   ", 2:"turtle  ", 3:"camera 2", 4:"arm 2   "}
global state_machine    ;   state_machine = {0:"off      ", 1:"on       ", 2:"moving   ", 3:"stop & on", 4:"error    "}

# machine(name) _ 0:camera1 / 1:arm1 / 2:turtle / 3:camera2 / 4:arm2 / 5:for empty
# global state_table  ;   init color = black
global label_key    ;   label_key = {0:3, 1:6, 2:9, 3:12, 4:15} # +1:sign , +2:course

# sign _ 0:black(off) / 1:green(on) / 2:orange(moving) / 3:gray(stop&on) / 4:red(error)
#global sign_color ;   sign_color = {0:"black", 1:"green", 2:"orange", 3:"gray", 4:"red"}
global sign_color ;   sign_color = {0:"black", 1:"SpringGreen3", 2:"orange", 3:"gray70", 4:"red"}


########################################################################    WINDOW CONFIG

# 가장 상위레벨의 가장 기본적인 윈도우 창 생성
global window   ;   window = tkinter.Tk()       
window.title(" ★ Turtle's dream ★ ")
window.geometry("669x650+100+100")      # pixel : 너비x높이+x좌표+y좌표 / 윈도우 창 , 초기 위치초기  # 580->650
window.resizable(True, True)          # 좌우, 상하 윈도우 창의 크기 조절 가능여부
window.configure(bg="antiquewhite")

# label type - list[19] 생성
global gui_label ; gui_label = [0 for i in range(19)] 

# 1*1px 투명 img
pixelVirtual = tkinter.PhotoImage(width=1, height=1) 

# font 설정
global font_title   ;   font_title = tkinter.font.Font(family="맑은 고딕", size=30, weight="bold") 
global font_button  ;   font_button = tkinter.font.Font(family="맑은 고딕", size=25, weight="bold") 
global font_name    ;   font_name = tkinter.font.Font(family="맑은 고딕", size=16, weight="bold")
global font_etc     ;   font_etc = tkinter.font.Font(family="맑은 고딕", size=16) 


########################################################################    GUI FUNCTION

# top label 생성
def WIN_TOP() :
    gui_label[0] = tkinter.Label(window, text=" TURTLE ●_● TURTLE ", font=font_title, image=pixelVirtual, compound="c", width=toplabel_w, height=toplabel_h, bg="springgreen4", fg="white", relief="raised")    

# button 생성 -> START / STOP
def WIN_BUTTON() :
    gui_label[1] = tkinter.Button(window, text="▶ START", font=font_button, image=pixelVirtual, compound="c", width=button_w, height=button_h, bg="blue", fg="white", command=pub_start)
    gui_label[2] = tkinter.Button(window, text="⏹ STOP", font=font_button, image=pixelVirtual, compound="c", width=button_w, height=button_h, bg="red", fg="white", command=pub_stop)

# NAME label 생성
def WIN_NAME() :
    tmp = 3
    gui_label[tmp*1] = tkinter.Label(window, text="CAMERA1", font=font_name, image=pixelVirtual, compound="c", width=name_w, height=name_h, fg="black", relief="raised")
    gui_label[tmp*2] = tkinter.Label(window, text="ARM1", font=font_name, image=pixelVirtual, compound="c", width=name_w, height=name_h, fg="black", relief="raised")
    gui_label[tmp*3] = tkinter.Label(window, text=" TURTLE", font=font_name, image=pixelVirtual, compound="c", width=name_w, height=name_h, fg="black", relief="raised")
    gui_label[tmp*4] = tkinter.Label(window, text="CAMERA2", font=font_name, image=pixelVirtual, compound="c", width=name_w, height=name_h, fg="black", relief="raised")
    gui_label[tmp*5] = tkinter.Label(window, text="ARM2", font=font_name, image=pixelVirtual, compound="c", width=name_w, height=name_h, fg="black", relief="raised")

# SIGN label 생성
def WIN_SIGN() :
    tmp = 3
    gui_label[tmp*1+1] = tkinter.Label(window, text="●", font=font_title, image=pixelVirtual, compound="c", width=sign_w, height=sign_h, fg=sign_color[machine_msg[0].m_state], relief="raised")
    gui_label[tmp*2+1] = tkinter.Label(window, text="●", font=font_title, image=pixelVirtual, compound="c", width=sign_w, height=sign_h, fg=sign_color[machine_msg[1].m_state], relief="raised")
    gui_label[tmp*3+1] = tkinter.Label(window, text="●", font=font_title, image=pixelVirtual, compound="c", width=sign_w, height=sign_h, fg=sign_color[machine_msg[2].m_state], relief="raised")
    gui_label[tmp*4+1] = tkinter.Label(window, text="●", font=font_title, image=pixelVirtual, compound="c", width=sign_w, height=sign_h, fg=sign_color[machine_msg[3].m_state], relief="raised")
    gui_label[tmp*5+1] = tkinter.Label(window, text="●", font=font_title, image=pixelVirtual, compound="c", width=sign_w, height=sign_h, fg=sign_color[machine_msg[4].m_state], relief="raised")

# ETC label 생성
def WIN_COURSE() :
    tmp = 3
    gui_label[tmp*1+2] = tkinter.Label(window, text=" ", font=font_etc, image=pixelVirtual, compound="c", width=course_w, height=course_h, fg="black", relief="raised")
    gui_label[tmp*2+2] = tkinter.Label(window, text=" ", font=font_etc, image=pixelVirtual, compound="c", width=course_w, height=course_h, fg="black", relief="raised")
    gui_label[tmp*3+2] = tkinter.Label(window, text=" ", font=font_etc, image=pixelVirtual, compound="c", width=course_w, height=course_h, fg="black", relief="raised")
    gui_label[tmp*4+2] = tkinter.Label(window, text=" ", font=font_etc, image=pixelVirtual, compound="c", width=course_w, height=course_h, fg="black", relief="raised")
    gui_label[tmp*5+2] = tkinter.Label(window, text=" ", font=font_etc, image=pixelVirtual, compound="c", width=course_w, height=course_h, fg="black", relief="raised")

# STATE_VIEW 생성
def WIN_STATEVIEW() :  
    gui_label[18] = tkinter.Button(window, text="● STATE VIEW ●", font=font_name, image=pixelVirtual, compound="c", width=toplabel_w-23, height=name_h, bg="black", fg="white", command=state_view)

# 위치 잡아주기     .place = pixel
def WIN_POSITION() :
    # setting
    gui_label[0].place(x=top_x, y=top_y)    # top
    gui_label[1].place(x=btn_x1, y=btn_y1)      ;   gui_label[2].place(x=btn_x2, y=btn_y1)   # button
    gui_label[3].place(x=label_x1, y=label_y1)  ;   gui_label[4].place(x=label_x2, y=label_y1)    ;   gui_label[5].place(x=label_x3, y=label_y1)     # name sign course
    gui_label[6].place(x=label_x1, y=label_y2)  ;   gui_label[7].place(x=label_x2, y=label_y2)    ;   gui_label[8].place(x=label_x3, y=label_y2)
    gui_label[9].place(x=label_x1, y=label_y3)  ;   gui_label[10].place(x=label_x2, y=label_y3)   ;   gui_label[11].place(x=label_x3, y=label_y3) 
    gui_label[12].place(x=label_x1, y=label_y4) ;   gui_label[13].place(x=label_x2, y=label_y4)   ;   gui_label[14].place(x=label_x3, y=label_y4) 
    gui_label[15].place(x=label_x1, y=label_y5) ;   gui_label[16].place(x=label_x2, y=label_y5)   ;   gui_label[17].place(x=label_x3, y=label_y5) 
    gui_label[18].place(x=btn_x1, y=btn_y2)     ;   # state_view

# init : 초기설정..
def MACHINE_INIT() :
    WIN_TOP()
    WIN_BUTTON()
    WIN_NAME()
    WIN_SIGN()
    WIN_COURSE()
    WIN_STATEVIEW()
    WIN_POSITION()
    return

# SIGN_UPDATE
def WIN_SIGN_UPDATE() :     # sign
    gui_label[4].place(x=label_x2, y=label_y1)
    gui_label[7].place(x=label_x2, y=label_y2)
    gui_label[10].place(x=label_x2, y=label_y3)
    gui_label[13].place(x=label_x2, y=label_y4)
    gui_label[16].place(x=label_x2, y=label_y5)

# COURSE_UPDATE
def WIN_COURSE_UPDATE() :     # course
    gui_label[5].place(x=label_x3, y=label_y1)
    gui_label[8].place(x=label_x3, y=label_y2)
    gui_label[11].place(x=label_x3, y=label_y3)
    gui_label[14].place(x=label_x3, y=label_y4)
    gui_label[17].place(x=label_x3, y=label_y5) 


########################################################################    STATE VIEW

def state_view() : 
    os.system('clear')
    now = datetime.now()
    print(" ■■■■■■■■■■■■■■■■  state_view ■■■■■■■■■■■■■■■■ ")
    print(" --- ", now.year, ".", now.month, ".", now.day, " - ", now.hour, ":", now.minute, ":", now.second, " --- ")
    for i in range(5) : 
        print(" [" + str(machine_msg[i].machine) + "] " + name_machine[i] + " ---> [" + str(machine_msg[i].m_state) + "] " + state_machine[machine_msg[i].m_state] + " : course(" + str(machine_msg[i].t_course) + ")")
    print(" ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■ ")
    return


########################################################################    SIGNAL CHECK

# n초 이상 안오면, off 로 바꿔주기 ?# n초 이상 안오면, off 로 바꿔주기 ?# n초 이상 안오면, off 로 바꿔주기 ?# n초 이상 안오면, off 로 바꿔주기 ?# n초 이상 안오면, off 로 바꿔주기 ?
# n초 이상 안오면, off 로 바꿔주기 ?
# 신호 안오는 것만 gray 로 변경되게 할 것

def time_stamp(i) : 
    current_time = rospy.Time.now()      # 현재시간 
    machine_time = machine_msg[i].stamp  # machine_msg[i].stamp 에 있는 시간 
    return current_time.secs - machine_time.secs    # secs 단위로 빼서 return (확인) 
    # start , end time -> int sec nsec 

# 5초마다 signal 을 check 해주는 것
def no_signal() :
    while not v_signal_check.is_set() :     # 1
        for i in range(5) :         # 전체 돌면서 마지막으로 msg 수신한 시간을 체크 , 5초가 넘어가면 stop&on 상태로 변경
            if machine_msg[i].m_state is not 0 :        # m_state가 0 이 아니라면 = 상태값을 받았었다면 확인
                var = time_stamp(i)     # time_stamp() 로 i 머신의 topic 이 정상수신되고 있는지 확인
                if var > 5 :    # msg 수신 후 5초 지났다면,
                    machine_msg[i].m_state = 4    # 3:gray(stop&on)     # 4:red(error)
                    machine_msg[i].t_course = 0
                    sub_m_state(machine_msg[i])     # sign
                    sub_t_course(machine_msg[i])    # course
                    print(" >>>>>>>>>> ---! machine[" + str(i) + "] no signal !--- <<<<<<<<<< ")
        time.sleep(5)
    return

t_signal_check = 0  # signal 확인 용 thread 생성여부 확인

def sig_check_start() :
    global t_signal_check
    if t_signal_check is 0 :
        print(" * signal_check Thread start * {0}".format(t_signal_check))
        t_signal_check = threading.Thread(target=no_signal) # , args=(1, 100000))
        global v_signal_check   ;   v_signal_check = threading.Event() # 1
        t_signal_check.start()
    return

def sig_check_stop() :
    global t_signal_check
    if t_signal_check is not 0 :
        print(" * signal_check Thread stop * {0}".format(t_signal_check))
        v_signal_check.set()    # threading.Event() 이벤트 발생시킴
        t_signal_check.join()
        t_signal_check = 0
    return


######################################################################## publish 

# START button -> msg.button 1(start) publish & signal_check
def pub_start() :
    now = datetime.now()
    msg = GuiMsg()    
    msg.button = 1
    pub_button.publish(msg)
    os.system('clear')
    print(" ---------------- start _ publish [ ", now.hour, ":", now.minute, ":", now.second, " ] ---------------- ")
    sig_check_start()   # signal check start
    return

# STOP button -> msg.button 0(stop) publish 
# STOP 은 모두가 구독하고 있어야 할 것
def pub_stop() :
    msg = GuiMsg()
    msg.button = 0
    pub_button.publish(msg)
    print("-------------- stop _ publish --------------")
    sig_check_stop()    # signal check stop
    return

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # subscriber

# 전달받은 msg를 복사하면서 stamp(time) 찍기 _ 가장 최근 받은 해당 머신의 msg를 저장, 지속적으로 msg 받고있는지 확인하는 용도
def copy_msg(msg) :
    machine_msg[msg.machine] = msg  # 통째로 복사
    machine_msg[msg.machine].stamp = rospy.Time.now()
    return

# sign _ 0:black(off) / 1:green(on) / 2:orange(moving) / 3:gray(stop&on) / 4:red(error)
def sub_m_state(msg) :
    gui_label[label_key[msg.machine]+1].config(fg = sign_color[machine_msg[msg.machine].m_state])
    WIN_SIGN_UPDATE()
    return

# int32 t_course  # turtlebot _ 1:arm1 / 2:arm2
def sub_t_course(msg) :
    machine_table = {0:label_key[0]+2, 1:label_key[1]+2, 2:label_key[2]+2, 3:label_key[3]+2, 4:label_key[4]+2} # 5, 8, 11, 14, 17
    if msg.machine == 2 :       # turtle
        if msg.t_course == 0 :
            gui_label[machine_table[2]].config(text=" ")
        elif msg.t_course == 1 : 
            gui_label[machine_table[2]].config(text=" arm2 → arm1 ")
        elif msg.t_course == 2 : 
            gui_label[machine_table[2]].config(text=" arm1 → arm2 ")
    else :
        if msg.t_course == 0 :
            gui_label[machine_table[msg.machine]].config(text=" ")
        elif msg.t_course == 1 : 
            gui_label[machine_table[msg.machine]].config(text=" t_course = 1 ")
        elif msg.t_course == 2 : 
            gui_label[machine_table[msg.machine]].config(text=" t_course = 2 ")       
    WIN_COURSE_UPDATE()
    return

####### ####### #######     add
def check_change_state(msg) : 
    if msg.m_state is not machine_msg[msg.machine].m_state :
        now = datetime.now()
        print("[", now.hour, ":", now.minute, ":", now.second, "]       change machine / "+ name_machine[msg.machine] + " : " + state_machine[machine_msg[msg.machine].m_state] + " -> " +  state_machine[msg.m_state])

# uint8 : machine
# msg받은 machine의 state라벨.config 로 state_table 참고하여 state 색상 변경
def sub_m_update(msg) : 
    if msg.machine < 5 :        # 받은 topic 이 machine 0~4 라면, update / test topic (5) 이라면 그냥 끝내기
        check_change_state(msg) # add
        copy_msg(msg)       # 받은 msg를 machine_msg[msg.machine] table 에 복사 & 현재 시간을 machine_msg[msg.machine].stamp 에 저장
        sub_m_state(msg)    # 전달받은 상태값을 window에 보여줌
        sub_t_course(msg)   # 상태값 바뀌었으니까 , 그에 맞게 course label 도 바꿔주기
    return

######################################################################## node_init

# node_init
rospy.init_node("gui_turtle", anonymous=True) # , anonymous=True

# publisher     rospy.Publisher("topic", datatype, option)
pub_button = rospy.Publisher("StartStop_Topic", GuiMsg, queue_size=100)

# subscriber    rospy.Subscriber("topic", datatype, function, queue_size=n)
sub_label = rospy.Subscriber('Gui_Topic', GuiMsg, sub_m_update, queue_size=1000)

rate = rospy.Rate(1)
rate.sleep()

########################################################################    START

MACHINE_INIT()

#    THREAD
t_window = threading.Thread(target=window.mainloop()) # , args=(1, 100000))
t_window.start()

# rospy.spinOnce()
rospy.spin()

# 프로그램이 종료될 때, 쓰레드를 정리해줌 !
t_window.join()

########################################################################


