0. ubuntu 16.04, ROS kinetic, opencv가 설치된 컴퓨터 혹은 노트북 2대, (이하 PC1, PC2)

   ubuntu 16.04, ROS kinetic, python3이 설치된 컴퓨터 혹은 노트북 1대를 준비한다. (이하 Master PC)

   터틀봇3 와플파이에 라즈베리파이 os (ver. stretch), ROS kinetic가 설치된 라즈베리 파이 3b+와 OpenCR 보드를 준비한다. (이하 라즈베리 파이)




1. Master PC 터미널에

   $ rosrun GUI_turtle gui_multiple.py
   $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
   $ map_file:=$HOME/map.yaml
   $ roslaunch turtle_node turtle_node.launch

   를 입력한다.




2. 라즈베리 파이 터미널에

   $ roslaunch turtlebot3_bringup turtlebot3_robot.launch

   를 입력한다.




3. PC1에 카메라1, U-ARM1을 연결하고 터미널에 

   $ sudo chmod 666 /dev/ttyACM0
   $ roslaunch iwh_arm iwh_launch_1.launch

   를 입력한다.




4. PC2에 카메라2, U-ARM2을 연결하고 터미널에 

   $ sudo chmod 666 /dev/ttyACM0
   $ roslaunch iwh_arm iwh_launch_2.launch

   를 입력한다.


