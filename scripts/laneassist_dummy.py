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

UDP_IP = "192.168.0.90"
UDP_PORT = 10002



def callback(data):
    rospy.loginfo("offset: " + str(int(data.data)))

    offset = data.data
    
    # p regulator
    steering_angle = 0.15 * offset
    # TODO PID -> steering angle

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