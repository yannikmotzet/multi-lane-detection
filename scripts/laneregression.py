#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import random
import math

picHeight = 960
picWidth = 1280
poly_degree = 2
canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
data_function_collected = ""


def draw_circles(points):
    # r, g, b = random.randint(0, 255), random.randint(
        # 0, 255), random.randint(0, 255)
    r, g, b = 0, 0, 0
    # size ergibt sich aus x + y --> besser machen
    for i in range(int(points.size / 2)):
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1, (r, g, b), -1)


def calcluate_distance_between_points(x1, y1, x2, y2):
    x_diff = x2 - x1
    y_diff = y2 - y1
    return math.sqrt(x_diff**2 + y_diff**2)

def sort_function_points(function_points):
    number_of_points = int(function_points.size / 2)
    function_points = np.reshape(function_points, (number_of_points, 2))
    # https://stackoverflow.com/questions/2828059/sorting-arrays-in-numpy-by-column?noredirect=1&lq=1
    # function_points = np.sort(function_points.view('i4,i4'), order=['f1','f0'], axis=0).view(np.int)
    function_points.view('i4,i4').sort(order=['f1','f0'], axis=0)
    return function_points

def build_function(function_points_sorted):
    number_of_points = int(function_points_sorted.size / 2)
    value_table = np.zeros((number_of_points, 3))
    value_table[:, 1:] = function_points_sorted
    value_table[0, 0] = 0
    for i in range(1, number_of_points):
        value_table[i, 0] = value_table[i-1, 0] + calcluate_distance_between_points(value_table[i, 1], value_table[i, 2], value_table[i-1, 1], value_table[i-1, 2])
    # regression for x
    x_function = np.polyfit(value_table[:, 0], value_table[:, 1], poly_degree)
    # regression for y
    y_function = np.polyfit(value_table[:, 0], value_table[:, 2], poly_degree)
    return [x_function, y_function]

    

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # clusters
    global canvas
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    message = ""

    cluster = data.data.split(";")
    number_of_cluster = len(cluster) - 1

    # go through clusters
    for n in range(number_of_cluster):
        fields = cluster[n].split(",")
        number_of_points = int((len(fields) - 1) / 2)
        points = np.zeros((number_of_points, 2))
        cluster_y_value_min = 960
        cluster_y_value_max = 0

        # go through points of cluster
        for i in range(len(fields) - 1):   # last field is empty
            # x-value
            if (i % 2 == 0):
                points[(int(i/2)), 0] = fields[i]
            # y-value
            else:
                points[(int((i-1)/2)), 1] = fields[i]

        # convert string to int
        points = points.astype(int)
        # draw points on canvas
        draw_circles(points)

        # build function string for message (polydegree, start point, end point)
        function_string = "hello"
        message = message + function_string




        function_points = cv2.approxPolyDP(points, 2, False)
        cv2.drawContours(canvas, function_points, -1, (0, 0, 255), 5)

        function_points_sorted = sort_function_points(function_points)
        function = build_function(function_points_sorted)

        for i in range(0, 1500, 2):
            x = np.polyval(function[0], i)
            y = np.polyval(function[1], i)
            cv2.circle(canvas, (int(x), int(y)), 1, (0, 255, 0), -1)


        print(function_points)
        # print(function_points.shape)
        print(sort_function_points(function_points))




    # talker(str(poly_degree) + ";" + message)
    # perspective_transformation()
    cv2.imshow("points", canvas)
    cv2.waitKey(8000)
    cv2.destroyAllWindows()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laneregression', anonymous=True)

    rospy.Subscriber("chatter_cluster", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker(data_function):
    pub = rospy.Publisher('chatter_function', String, queue_size=10)
    # rospy.init_node('talker_function', anonymous=True)
    rate = rospy.Rate(10)  # hz
    if not rospy.is_shutdown():
        rospy.loginfo(data_function)
        pub.publish(data_function)
        rate.sleep()


if __name__ == '__main__':
    listener()
