"""
Personal Robotics Laboratory - Imperial College London, 2021
Authors:
    - Rodrigo Chacon Quesada (rac17@ic.ac.uk)
    - Haining Luo (haining.luo18@imperial.ac.uk)
    - Cedric Goubard (c.goubard21@imperial.ac.uk)


Inspired from https://github.com/redragonx/can2RNET
Original code was modified to:
    - make it work with ROS Noetic
    - adapt to the specificities of our joysticks
    - work in our dockerised setup

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
from ctypes import ArgumentError
from time import sleep

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from typing import Tuple
import threading
from typing import List 
import pygame


pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check for joystick availability
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    quit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

class robot_interface:

    def __init__(self,):
        
        ######################
        ##### Substribers #####
        ######################
        topic_name = "/VR/joy"
        self.sub_VR_joy = rospy.Subscriber(topic_name, Joy, self.call_back_VR_joy, queue_size=1)
        rospy.loginfo("VR joystick subscriber created")
        
        ### publish back to the wheelchair
        ## ROS publisher to send joystick data
        self.robot_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        rospy.loginfo('Final velocity command publisher created')
        
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        
        self.move = True
        

       
    
    #########################
    ##### Methods #######
    ######################### 
    def getKey(self):
        linear = False
        for event in pygame.event.get():
            
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 1:
                    linear = True
                else:
                    linear = False
            
                return (linear, event.value)
        return (False, 0)


    def run(self):
        while not rospy.is_shutdown():
            try:
                '''
                DO STUFF HERE to process
                '''
                linear, value = self.getKey()
                if linear:
                    self.cmd_vel.linear.x = value
                    self.cmd_vel.angular.z = 0
                else:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = value

                self.robot_pub.publish(self.cmd_vel)
                rospy.loginfo("Publishing velocity command")
                rospy.loginfo(self.cmd_vel)
                    
                    
                    
                    
            except KeyboardInterrupt:
                self.shutdown()
                rospy.loginfo("Connection Off")
                rospy.loginfo("Socket down")
                break
            
    def shutdown(self):
        rospy.loginfo("Closing robot_interface node")
        rospy.signal_shutdown()
    
    
    ## ROS PUBLISHERS --> Define ROS publishers here

    ## ROS CALLBACKS

    ## should be called when we receive a joystick message
    def call_back_VR_joy(self, joy_msg: Joy):
        ## get the joystick command
        self.cmd_vel.linear.x = joy_msg.axes[0]
        self.cmd_vel.angular.z = joy_msg.axes[1]
        
        ## NOTE: need to ensure linear.x and angular.z are set to 0 (from server function) when the joystick is not being used
        
        
        

if __name__ == "__main__":

    rospy.sleep(0.5)

    rospy.init_node('robot_interface', anonymous=True)

    rmv = robot_interface()

    try:
        rmv.run()
    except KeyboardInterrupt:
        rmv.shutdown()
        rospy.loginfo("CAN disconnected")
        rospy.loginfo("Connection Off")
        rospy.loginfo("Socket down")

