#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
import cv2
import random
import math

picHeight = 960
picWidth = 1280
poly_degree = 1
canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
data_function_collected = ""


def print_circles(points):
    r, g, b = random.randint(0, 255), random.randint(
        0, 255), random.randint(0, 255)
    # size ergibt sich aus x + y --> besser machen
    for i in range(int(points.size / 2)):
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1, (r, g, b), -1)


def calcluate_distance_between_points(x1, y1, x2, y2):
    x_diff = x2 - x1
    y_diff = y2 - y1
    return math.sqrt(x_diff**2 + y_diff**2)


def perspective_transformation():
    global canvas
    # illustrate vertices for perspective transformation
    # point top left
    cv2.circle(canvas, (400, 0), 5, (255, 0, 0), -1)
    # point top right
    cv2.circle(canvas, (picWidth-400, 0), 5, (0, 255, 0), -1)
    # point bottom left
    cv2.circle(canvas, (50, picHeight), 5, (255, 0, 0), -1)
    # point bottom right
    cv2.circle(canvas, (picWidth-50, picHeight), 5, (0, 0, 255), -1)

    # perspective transformation
    pts1 = np.float32([[400, 0], [picWidth-400, 0],
                       [0, picHeight], [picWidth, picHeight]])
    pts2 = np.float32(
        [[0, 0], [picWidth, 0], [400, picHeight], [picWidth-400, picHeight]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)

    canvas = cv2.warpPerspective(canvas, matrix, (picWidth, picHeight))


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # clusters
    global canvas
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    message = ""

    cluster = data.data.split(";")
    number_of_cluster = len(cluster) - 1
    function_list = np.zeros((2, 2, number_of_cluster))

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
                # find min und max y-value
                if int(fields[i]) < cluster_y_value_min:
                    cluster_y_value_min = int(fields[i])
                elif int(fields[i]) > cluster_y_value_max:
                    cluster_y_value_max = int(fields[i])

        # convert string to int
        points = points.astype(int)
        # draw points on canvas
        print_circles(points)

        # calculate function (y, x)
        function = np.polyfit(points[:, 1], points[:, 0], poly_degree)

        # # draw all points of function line (y, x)
        # for i in range(int(cluster_y_value_min), int(cluster_y_value_max)):
        #     cv2.circle(canvas, (int(np.polyval(function, i)), i), 1, (0, 0, 0), -1)

        # print line from start to end point of function
        function_start_point = np.polyval(function, cluster_y_value_min)
        function_end_point = np.polyval(function, cluster_y_value_max)
        # cv2.line(canvas, (int(function_start_point), cluster_y_value_min) , (int(function_end_point), cluster_y_value_max),(0, 0, 0) , 1)

        # append to function list
        function_list[0, 0, n] = int(function_start_point)
        function_list[0, 1, n] = int(cluster_y_value_min)
        function_list[1, 0, n] = int(function_end_point)
        function_list[1, 1, n] = int(cluster_y_value_max)
        function_list = function_list.astype(int)

        # build function string for message (polydegree, start point, end point)
        function_string = (str(int(function_start_point)) + ":" + str(cluster_y_value_min) +
                           "," + str(int(function_end_point)) + ":" + str(cluster_y_value_max) + ";")
        message = message + function_string

    # look for dashed lines
    distance_treshold = 200
    function_list_dashed = np.zeros((number_of_cluster))
    for i in range(number_of_cluster):
        for k in range(number_of_cluster):
            # check for y-distance of end, start and length
            if (function_list[1,1,i] - function_list[0,1,k]) < 80 and (calcluate_distance_between_points(function_list[1, 0, i], function_list[0, 0, i], function_list[1, 1, i], function_list[0, 1, i])) < 400:
                function_list_dashed[i] = 1
                function_list_dashed[k] = 1


    # draw functions
    for i in range(number_of_cluster):
        if function_list_dashed[i] == 0:
            cv2.line(canvas, (function_list[0, 0, i], function_list[0, 1, i]), (
                function_list[1, 0, i], function_list[1, 1, i]), (0, 0, 0), 1)
        # else:
        #     cv2.line(canvas, (function_list[0, 0, i], function_list[0, 1, i]), (
        #         function_list[1, 0, i], function_list[1, 1, i]), (255, 0, 0), 1)

    talker(str(poly_degree) + ";" + message)
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
