# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/vishnu/Project/Go1_FollowMe/04_Git_ws/go1_followmeforwalk/devel;/home/vishnu/Project/Go1_FollowMe/03_catkin_ws/devel;/home/vishnu/Documents/rl_gym_ros/devel;/home/vishnu/Documents/my_robotarm/devel;/home/vishnu/Documents/gazebo_prac/devel;/home/vishnu/catkin_ws/devel;/home/vishnu/ws_moveit/devel;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/vishnu/Project/Go1_FollowMe/04_Git_ws/go1_followmeforwalk/devel/.private/go1_slam/env.sh')

output_filename = '/home/vishnu/Project/Go1_FollowMe/04_Git_ws/go1_followmeforwalk/build/go1_slam/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
