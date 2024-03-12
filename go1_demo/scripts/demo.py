#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2024, MYBOTSHOP GmbH, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of MYBOTSHOP GmbH nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission
# of MYBOTSHOP GmbH.

import time
import rospy
import moveit_commander
from threading import Thread
from open_manipulator_msgs.srv import SetJointPosition  
from open_manipulator_msgs.msg import JointPosition

class MmpDemo(object):
    def __init__(self):
        rospy.init_node('go1_demo', anonymous=False)     
        self.thread = None
        self.go1_move_group  = moveit_commander.MoveGroupCommander("arm")

    def create_thread(self, func, args, is_join=False):
        self.thread = Thread(target=func, args=args)
        self.thread.daemon = True
        self.thread.start()
        if is_join:
            self.thread.join()
        time.sleep(1)

    def colorize(self, text, color):
        color_codes = {
            'green': '\033[92m',
            'yellow': '\033[93m',
            'orange': '\033[38;5;208m', 
            'blue': '\033[94m',
            'red': '\033[91m'
        }
        return color_codes[color] + text + '\033[0m'
    
    def gripper_control(self, pos=0.01):
        '''
        Open gripper  =  0.01
        Close gripper = -0.01
        '''
        rospy.wait_for_service('/go1_robotis/goal_tool_control')
        try:
            goal_tool_control = rospy.ServiceProxy('/go1_robotis/goal_tool_control', SetJointPosition)
            joint_position = JointPosition
            joint_position.joint_name = ['gripper']
            joint_position.position = [pos]
            joint_position.max_accelerations_scaling_factor = 0.0
            joint_position.max_velocity_scaling_factor = 0.0
            response = goal_tool_control('gripper', joint_position, 0.0)
        except rospy.ServiceException as e:
            rospy.logwarn(self.colorize('Gripper service call failed: ', 'red'), e)

    def move_arm_predefined_positions(self, predef):
        # The name can be a name previlusy remembered with remember_joint_values() or a configuration specified in the SRDF. 
        self.go1_move_group.set_named_target(predef)
        self.go1_move_group.go(wait=True)
        self.go1_move_group.stop()
        time.sleep(3)

    def move_arm_jointstate(self, waypoint, go_to_home=False):
        self.go1_move_group.go(waypoint, wait=True)
        self.go1_move_group.stop()

    def run(self):

        rospy.loginfo(self.colorize('Moveit Teleop Active', 'orange'))
        rospy.loginfo(self.colorize('s -> Stand Ready Position', 'yellow'))
        rospy.loginfo(self.colorize('h -> Home Position for closing arm', 'yellow'))
        rospy.loginfo(self.colorize('l -> Move Arm to the left', 'yellow'))
        rospy.loginfo(self.colorize('r -> Move Arm to the right', 'yellow'))
        rospy.loginfo(self.colorize('p -> Front Pick Position', 'yellow'))
        rospy.loginfo(self.colorize('g -> Open Gripper', 'yellow'))
        rospy.loginfo(self.colorize('f -> Close Gripper', 'yellow'))
        rospy.loginfo(self.colorize('q -> Quit', 'red'))

        while not rospy.is_shutdown():

            event =  raw_input()

            if event =='s':
                self.move_arm_predefined_positions('stand')

            if event =='h':
                self.move_arm_predefined_positions('home')

            if event =='p':
                self.move_arm_predefined_positions('pick')
            
            if event =='l':
                self.move_arm_predefined_positions('left')

            if event =='r':
                self.move_arm_predefined_positions('right')

            if event =='g':
                self.gripper_control(0.01)
            
            if event =='f':
                self.gripper_control(-0.01)

            if event =='q':
                break
            
        # rospy.loginfo(self.colorize('Executing pick Position', 'orange'))
        # self.move_arm_predefined_positions('pick')
        # self.gripper_control(-0.01)

        # rospy.loginfo(self.colorize('Executing custom Position', 'yellow'))
        # self.create_thread(self.gripper_control, (-0.01, ))
        # self.move_arm_jointstate([-0.608, -0.44, 0.14, 0.77])
        # self.thread.join()
        # time.sleep(3)

        rospy.loginfo(self.colorize('Demo Complete', 'green'))
        
if __name__ == '__main__':
    demo = MmpDemo()
    demo.run()
