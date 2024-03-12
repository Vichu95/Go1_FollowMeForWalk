# Pass through to the main ROS workspace of the system.
# source /opt/ros/noetic/setup.bash
# source /home/unitree/ros_ws/devel/setup.bash

export ROBOT_SETUP=/etc/ros/setup.bash
export GO1_LOGITECH=1
export GO1_GPS_ODOM=false

# ROS Environment Variables
# export ROS_MASTER_URI=http://192.168.123.14:11311/
# export ROS_IP=192.168.123.14
# export ROS_HOSTNAME=192.168.123.14

bold="\033[1m"
yellow="\033[1;33m"
green="\033[0;32m"
red="\033[0;31m"
reset="\033[0m"

go1_sit_func(){
echo -e "${bold}${yellow}GO1 Sitting${reset}"
rosservice call /go1/set_mode "mode: 5
gait_type: 0"
echo -e "${bold}${yellow}GO1 Sitting Complete${reset}"
}

go1_stand_func(){
echo -e "${bold}${yellow}GO1 Standing${reset}"
rosservice call /go1/set_mode "mode: 6
gait_type: 0"
rosservice call /go1/set_mode "mode: 2
gait_type: 1"
echo -e "${bold}${yellow}GO1 Standing Complete${reset}"
}

sit_func(){
echo -e "${bold}${yellow}GO1 Sitting${reset}"
rosservice call /set_mode "mode: 5
gait_type: 0"
echo -e "${bold}${yellow}GO1 Sitting Complete${reset}"
}

stand_func(){
echo -e "${bold}${yellow}GO1 Standing${reset}"
rosservice call /set_mode "mode: 6
gait_type: 0"
rosservice call /set_mode "mode: 2
gait_type: 1"
echo -e "${bold}${yellow}GO1 Standing Complete${reset}"
}

alias sit=sit_func
alias stand=stand_func
alias mmp_sit=go1_sit_func
alias mmp_stand=go1_stand_func