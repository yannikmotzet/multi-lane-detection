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
            self.Kp = 4
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
            self.P_element = self.Kp * offset * (-1)				                # Kp: Verstaerkungsfaktor - gain factor
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
    # rospy.loginfo("offset: " + str(data.data))

    # get offset value from message
    offset = data.data

    # P-Controller
    ####################################
    controller = P_Controller()
    controller.init_controller()
    steering_angle = controller.calc_controller(offset)

    # print(steering_angle)


    # UDP Message
    ####################################
    # build content for STEER UDP message (STEER,1,steering angle,) 
    # steering angle = 100 correspond to 30 degrees in truck
    message = "STEER,1," + str(steering_angle) + ","
    message_udp_steer = str.encode(message)
    
    # build content for SPEED UDP message (SPEED,1,velocity,acceleration,)
    default_speed = 25
    message_udp_speed = str.encode("SPEED,1," + str(default_speed)+",50,")


    # create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # send UDP message
    sock.sendto(message_udp_steer, (UDP_IP, UDP_PORT))
    sock.sendto(message_udp_speed, (UDP_IP, UDP_PORT))
    
    
def listener():

    rospy.init_node('laneassist_dummy', anonymous=True)

    rospy.Subscriber("laneregression_offset", Float32, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # initialize steering angle
    message ="STEER,1," + str(0)+","

    MESSAGE = str.encode(message)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    listener()