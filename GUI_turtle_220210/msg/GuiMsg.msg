# ~/catkin_ws/src/GUI_turtle/msg/GuiMsg.msg

time stamp      
uint8 button    # button _ 0:stop / 1:start
uint8 machine   # machine(name) _ 0:camera1 / 1:arm1 / 2:turtle / 3:camera2 / 4:arm2
uint8 m_state   # sign _ 0:black(off) / 1:green(on) / 2:orange(moving) / 3:gray(stop&on) / 4:red(error)
uint8 t_course  # turtlebot _ 1:arm1 / 2:arm2

# m_state -> state
# t_course -> message

