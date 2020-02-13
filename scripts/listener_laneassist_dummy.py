#!/usr/bin/env python
import rospy
from std_msgs.msg import String
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
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    message_data = data.data.split("|")
    log_message = "I recived " + str(int(len(message_data) - 1)) + " polynomial(s): \n" +  str(message_data[0 : int(len(message_data) - 1)])
    rospy.loginfo(log_message)

    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)

    # go through all lane functions
    for i in range(len(message_data) -1):
        line_data = message_data[i].split(";")
        print("line_data" + str(line_data))

        x_t = np.zeros((3))
        y_t = np.zeros((3))
        x_t_data = line_data[1].split(",")
        y_t_data = line_data[2].split(",")

        # build function
        for k in range(3):
            x_t[k] = float(x_t_data[k])
            y_t[k] = float(y_t_data[k])

        # draw line on canvas
        r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        for m in range(0, 1500, 2):                             
            x = np.polyval(x_t, m)
            y = np.polyval(y_t, m)
            cv2.circle(canvas, (int(x), int(y)), 1, (r, g, b), -1)
        
        print(str(x_t))
        print(str(y_t))


    # display canvas window
    cv2.imshow("listener_dummy", canvas)
    # cv2.imwrite('result.png', canvas)
    cv2.waitKey(8000)
    cv2.destroyAllWindows()

        

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener_laneassist_dummy', anonymous=True)

    rospy.Subscriber("chatter_function", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()