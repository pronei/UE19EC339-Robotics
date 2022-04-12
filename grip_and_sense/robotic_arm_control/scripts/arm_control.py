#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from time import sleep
from control_msgs.msg import JointControllerState
from time import sleep

class arm_control():

    def __init__(self):
        self.theta = 0
        self.gripper_pos = 0
        self.vert_pos = 0
        self.hor_pos = 0

        # initialize the publishers
        self.twist_joint_pub = rospy.Publisher('/janitor/twist_joint_position_controller/command', Float64, queue_size=10)
        self.pris_gripper_pub = rospy.Publisher('/janitor/prismatic_gripper_position_controller/command', Float64, queue_size=10)
        self.pris_vertical_pub = rospy.Publisher('/janitor/prismatic_vertical_position_controller/command', Float64, queue_size=10)
        self.pris_horizontal_pub = rospy.Publisher('/janitor/prismatic_horizontal_position_controller/command', Float64, queue_size=10)

        rospy.Subscriber('/janitor/twist_joint_position_controller/state', JointControllerState, self.twistCb)
        rospy.Subscriber('/janitor/prismatic_gripper_position_controller/state', JointControllerState, self.gripperCb)
        rospy.Subscriber('/janitor/prismatic_vertical_position_controller/state', JointControllerState, self.vertCb)
        rospy.Subscriber('/janitor/prismatic_horizontal_position_controller/state', JointControllerState, self.horCb)

        self.rate = rospy.Rate(10)
        sleep(3)

    def send_data_to_arm(self,theta, vert_dist, hor_dist):
        print('Up') # Vertical slider movement
        while abs(0.8 - self.vert_pos) > 0.06:
            self.pris_vertical_pub.publish(0.8)
            self.rate.sleep()
        
        print('Horizontal') # Vertical slider movement
        while abs(0 - self.hor_pos) > 0.001:
            self.pris_horizontal_pub.publish(0.0)
            self.rate.sleep()

        print('Rotating') # Rotation
        while abs(theta - self.theta) > 0.001:
            self.twist_joint_pub.publish(theta)
            self.rate.sleep()

        print('Horizontal') # Horizontal slider movement
        while abs(hor_dist - self.hor_pos) > 0.001:
            self.pris_horizontal_pub.publish(hor_dist)
            self.rate.sleep()
        
        print('Down')
        while abs(vert_dist - self.vert_pos) > 0.06:
            self.pris_vertical_pub.publish(vert_dist)
            self.rate.sleep()
    
    def reset_arm(self):
        self.send_data_to_arm(0, 0.8, 0)
        self.open_gripper()
    
    def close_gripper(self):
        self.pris_gripper_pub.publish(-0.4)
        sleep(3)
    
    def open_gripper(self):
        self.pris_gripper_pub.publish(0.0)
        sleep(3)

    def twistCb(self, data):
        self.theta = data.process_value
    
    def gripperCb(self, data):
        self.gripper_pos = data.process_value

    def vertCb(self, data):
        self.vert_pos = data.process_value
    
    def horCb(self, data):
        self.hor_pos = data.process_value
