# Quadruped Robotics Go1 ROS Wrapper

> [GO1 Docs](https://www.docs.quadruped.de/projects/go1/html/index.html)

- [X] GO1 Base driver
- [X] GO1 Description
- [X] GO1 Camera
- [X] GO1 Bringup
- [X] GO1 Viz
- [X] GO1 Control
- [X] GO1 Odom Navigation
- [X] GO1 Map Navigation

## Installation

- Clone the repository into your ROS1 workspace.

```bash
git clone https://github.com/MYBOTSHOP/qre_go1.git
```

- Go to your ROS workspace and install ROS dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

- Install GO1 Navigation Dependencies, first provide permissions and then execute

```bash
sudo chmod +x go1_install.bash && ./go1_install.bash
```

- For GO1 Navigation, you would additionally require the ROS repository of [yocs_velocity_smoother](https://github.com/yujinrobot/yujin_ocs)

- Build and source your ROS workspace

```bash
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

- For arm64 (GO1 Nvidia), sometimes the build is not detected so you can replace these lines in **CMakeLists.txt** in ``go1_base``:

```bash
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(SYSTEM_ARCHITECTURE amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(SYSTEM_ARCHITECTURE arm64)
endif()
```

- With the following

```bash
set(SYSTEM_ARCHITECTURE arm64)
```

### LAN Static Connection


To create a static connection in your own PC, in Ubuntu go to Settings → Network then click on ``+`` and create a new connection.

1. The first task is to go to **IPv4** and change the connection to **manual**.

2. The second task is to put the **Address** IP as **192.168.123.51** and the **Netmask** as **24**.


3. Click save and restart your network. 


4. After a successful connection let's check the host's local IP by typing in the Host PC's terminal.

```bash
ifconfig
```

5. This should show the host IP which was assigned in the above step. Now its time to check if we can ping the robot or not, to do so type in your host pc

```bash
ping 192.168.123.161
```

6. After a successful ping, it's time to access the robot. To access the robot you can type the following command:

```bash
ssh -X pi@192.168.123.161
ssh -X unitree@192.168.123.13
ssh -X unitree@192.168.123.14
ssh -X unitree@192.168.123.15
```

The password for all is 

```bash
123
```

Alternatively you can connect via WiFi. The default password for WiFi is **00000000**.

### WiFi Connection

- To gain access over wifi type out the command

```bash
ssh -X pi@192.168.12.1
123
```

- Then you can gain access to the Nvidia by typing in the pi terminal

```bash
ssh -X unitree@192.168.123.14
123
```

## GO1 Quick start

Provided that you connected via LAN, you can start and use the robot via:

```bash
sudo su
source <your_workspace>/devel/setup.bash
roslaunch go1_bringup bringup.launch
```

You can then teleop via

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

To visualize in RVIZ

```bash
roslaunch go1_viz view_moveit.launch
```

## GO1 & Open Manipulator 

Similarly, you can start the GO1 with the open manipulator with:

> **Note:** This must be run on the GO1 PC that has the open-manipulator connected to it!

```bash
roslaunch go1_bringup mmp_bringup.launch
```

Further information is available on [GO1 Docs](https://www.docs.quadruped.de/projects/go1/html/index.html)

- Incase of new setup of GO1 open manipulator provide permissions to the port via:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

## Open Manipulator Gripper

- Open manipulator can be controller via the Moveit arm controller group. The gripper can be controller via the services

- To open the gripper

```bash
rosservice call /go1_robotis/goal_tool_control "planning_group: 'gripper'
joint_position:
  joint_name:
  - 'gripper'
  position:
  - 0.01 
  max_accelerations_scaling_factor: 0.0
  max_velocity_scaling_factor: 0.0
path_time: 0.0"
```

- To close the gripper

```bash
rosservice call /go1_robotis/goal_tool_control "planning_group: 'gripper'
joint_position:
  joint_name:
  - 'gripper'
  position:
  - -0.01 
  max_accelerations_scaling_factor: 0.0
  max_velocity_scaling_factor: 0.0
path_time: 0.0"
```

- For different positions you may use any value between -0.01 to 0.01, where 0.01 is for opening and -0.01 is for closing.

## GO1 Camera

This requires ROS multi-machine setup. This package can be cloned to the 3 Nvidia boards on the GO1 `192.168.123.13`, `192.168.123.14`, `192.168.123.15` respectively.
In combination with robot upstart (not covered here yet) it should startup everytime the robot boots up.

### Multi machine setup


```bash
# Main PC
export ROS_MASTER_URI=http://192.168.123.1:11311
export ROS_HOSTNAME=192.168.123.1
export ROS_IP=192.168.123.1
```

```bash
# Nvidia 192.168.123.13 PC
export ROS_MASTER_URI=http://192.168.123.1:11311
export ROS_HOSTNAME=192.168.123.13
export ROS_IP=192.168.123.13
```

```bash
# Nvidia 192.168.123.14 PC
export ROS_MASTER_URI=http://192.168.123.1:11311
export ROS_HOSTNAME=192.168.123.14
export ROS_IP=192.168.123.14
```

```bash
# Nvidia 192.168.123.15 PC
export ROS_MASTER_URI=http://192.168.123.1:11311
export ROS_HOSTNAME=192.168.123.15
export ROS_IP=192.168.123.15
```
To setup multi-machine add the client pc details from the code above to the ``.bashrc`` file. 
The `IP 192.168.123.---` should be replaced with the Nvidia IP which can be found out with ``ifconfig``
e.g. ``13`` in this case.

> Note: The default camera driver auto-startup in each Nvidia needs to be disabled. 

### Disable Camera Driver startup

Run this on the PC and unclick the camera driver

```bash
gnome-session-properties
```

In some versions 
```bash
./Unitree/autostart/camerarosnode/cameraRosNode/kill.sh
```

## GO1 Navigation

### Odometry Navigation

For navigation based purely on the robots odometry first run the bringup


```bash
roslaunch go1_bringup bringup.launch
```

Next launch the odometry navigation

```bash
roslaunch go1_navigation odom_navi.launch
```

## GO1 Gazebo


- Launch Simulation

```bash
roslaunch go1_gazebo gazebo.launch
```

- Unpause simulations by pressing the play button

- Teleop GO1

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/go1/cmd_vel
```

- Gaits are untuned and can be changed via go1_gazebo/config/gait/gait.yaml

- Control OpenManipulator

```bash
roslaunch go1_gazebo open_manipulator_control.launch
```

- Press timerstart to use the GUI buttons. Also possible to control via the joint_command topics.

## GO1 Service Examples

- GO1 stand down position

```bash
rosservice call /set_mode "mode: 5
gait_type: 0" 
```

- GO1 force stand position

```bash
rosservice call /set_mode "mode: 6
gait_type: 0" 
```

- GO1 return to walk mode control

```bash
rosservice call /set_mode "mode: 2
gait_type: 1" 
```

> More information on [GO1 Modes](https://qiita.com/coder-penguin/items/63f1fc1da8958f46dadd)

## GO1 Service Examples with Open Manipulator

- GO1 stand down position

```bash
rosservice call /go1/set_mode "mode: 5
gait_type: 0" 
```

- GO1 force stand position

```bash
rosservice call /go1/set_mode "mode: 6
gait_type: 0" 
```

- GO1 return to walk mode control

```bash
rosservice call /go1/set_mode "mode: 2
gait_type: 1" 
```

## GO1 Multimachine with Nvidia 14 as ROS Master

### Nvidia 192.168.123.14 PC
```bash
export ROS_MASTER_URI=http://192.168.123.14:11311
export ROS_HOSTNAME=192.168.123.14
export ROS_IP=192.168.123.14
```

### Main PC
```bash
export ROS_MASTER_URI=http://192.168.123.14:11311
export ROS_HOSTNAME=192.168.123.51
export ROS_IP=192.168.123.51
```