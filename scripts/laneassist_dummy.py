#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from laneregression.msg import functionData, functionArray
import numpy as np
import cv2
import random

__author__ = 'Yannik Motzet'
__copyright__ = 'Copyright 2020, ZF DHBW Innovation Lab'
__version__ = '0.1.0'
__email__ = 'yannik.motzet@outlook.com'
__status__ = 'in development'

picHeight = 960
picWidth = 1280

def callback(data):
    rospy.loginfo("I recived " + str(int(len(data.functions))) + " polynomial(s).")

    functions = data.functions

    
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)

    for i in range(len(functions)):
        r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        x_t = [None] * 3
        y_t = [None] * 3
        x_t[0] = functions[i].a
        x_t[1] = functions[i].b
        x_t[2] = functions[i].c
        y_t[0] = functions[i].d
        y_t[1] = functions[i].e
        y_t[2] = functions[i].f
        for m in range(0, 1500, 2):                             
            x = np.polyval(x_t, m)
            y = np.polyval(y_t, m)
            cv2.circle(canvas, (int(x), int(y)), 1, (r, g, b), -1)
    
    # display canvas window
    cv2.imshow("laneassist_dummy", canvas)
    # cv2.imwrite('result.png', canvas)
    cv2.waitKey(2000)
    # cv2.destroyAllWindows()
    
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laneassist_dummy', anonymous=True)

    rospy.Subscriber("laneregression_functions", functionArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()