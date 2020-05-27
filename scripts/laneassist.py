#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import numpy as np
import cv2
import random
import math
import socket

__author__ = 'author'
__copyright__ = 'Copyright 2020, ZF DHBW Innovation Lab'
__version__ = '0.1.0'
__email__ = 'author@email.com'
__status__ = 'in development'

# IP and Port of Steering control unit (Arduino Mega / IPG TruckMaker)
UDP_IP = "192.168.0.90"
UDP_PORT = 10002


# Initialisierung des Reglers - Initialization of the controller
class P_Controller:
    def init_controller(self):		
            self.Kp = 0.5
            # self.offset = 0.0
            self.L_wheelbase = 325          # [mm]
            self.g_acceleration = 9.81      # [m/s^2] - acceleration of gravity
            self.speed_v = 0.0
            self.R_Curve = 0.0
            self.Setpoint = 0.0
            self.P_element = 0.0
            self.Steering_Angle = 0.0

# Berechnung des P-Glieds - Calculation of the proportional element
    def calc_controller(self, offset):
            self.P_element = self.Kp * offset				                # Kp: Verstaerkungsfaktor - gain factor
            return self.P_element

# Berechnung des Lenkwinkels - Calculation of the steering angle
            self.R_Curve = ((self.speed_v)**2/self.g_acceleration)/1000			# [(m/s)^2/(m/s^2)=m]/1000 = [mm] - curve radius			
            self.Steering_Angle = self.L_wheelbase/self.R_Curve					# [grad]

# Berechnung des Reglerwertes - Calculation of the Controller value
            if self.offset < 0:
                self.Steering_Angle > 0
                print("steering correction to the left")
            elif self.offset == 0:
                self.Steering_Angle = 0
                print("on ideal line")
            else:
                self.Steering_Angle < 0
                print("steering correction to the right")



def callback(data):
    rospy.loginfo("offset: " + str(int(data.data)))

    offset = data.data

    # PID-Controller
    ####################################
    controller = P_Controller()
    controller.init_controller()
    steering_angle = controller.calc_controller(offset)

    print(steering_angle)


    # UDP Message
    ####################################
    # build content for UDP message
    message = "STEER,1," + str(steering_angle)
    message_udp = str.encode(message)

    # create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # send UDP message
    sock.sendto(message_udp, (UDP_IP, UDP_PORT))
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laneassist_dummy', anonymous=True)

    rospy.Subscriber("laneregression_functions", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()