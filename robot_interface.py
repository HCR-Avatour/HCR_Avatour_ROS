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


class robot_interface:

    def __init__(self,):
        
        ######################
        ##### Substribers #####
        ######################
        topic_name = "/VR/joy"
        self.sub_VR_joy = rospy.Subscriber(topic_name, Joy, self.call_back_VR_joy, queue_size=1)
        rospy.loginfo("Robot joystick mode selected ")
        
        ## subscribe to a different (computer) topic
        back_sensor_topic_name = "/sensors/back_laser_scan"
        back_sensor_type = LaserScan
        self.sensor_sub = rospy.Subscriber(back_sensor_topic_name, back_sensor_type, self.call_back_back_sensor, queue_size=1)
        self.sensor_scan = []
        rospy.loginfo("Subscribed to " + str(back_sensor_topic_name))

       
    
    #########################
    ##### Methods #######
    ######################### 
        
    def run(self):

        while not rospy.is_shutdown():
            try:
                '''
                DO STUFF HERE to process
                '''
            except KeyboardInterrupt:
                self.shutdown()
                rospy.loginfo("Connection Off")
                rospy.loginfo("Socket down")
                break
            
    def shutdown(self):
        rospy.loginfo("Closing Socket")
        rospy.signal_shutdown()
    
    
    ## ROS PUBLISHERS --> Define ROS publishers here

    ## ROS CALLBACKS

    ## should be called when we receive a joystick message
    def call_back_VR_joy(self, joy_msg: Joy):
        ## get the joystick command
        joy_msg = joy_msg.axes
        

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

