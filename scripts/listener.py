#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import random

picHeight = 640
picWidth = 960

# white canvas
canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
cv2.imshow("points", canvas)



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    points = data.data.split(",")
    # print(points)
    cv2.destroyAllWindows()
    cv2.circle(canvas, (int(points[0]), int(points[1])), 5,(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)), -1)
    cv2.imshow("points", canvas)
    cv2.waitKey(200)


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
