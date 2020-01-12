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

def determine_cluster_start_end_points (cluster_with_points, number_of_cluster):
    cluster_start_end_points = np.zeros((2, 2, number_of_cluster))
    # determine start and end points
    for i in range(number_of_cluster):
        start_y = picHeight
        end_y = 0
        # determine min, max y-value
        for k in range(int(cluster_with_points[i].size / 2)):
            if (cluster_with_points[i])[k, 1] < start_y:
                start_y = (cluster_with_points[i])[k, 1]
            if (cluster_with_points[i])[k, 1] > end_y:
                end_y = (cluster_with_points[i])[k, 1]
        cluster_start_end_points[0, 1, i] = start_y
        cluster_start_end_points[1, 1, i] = end_y

        # determine x-value of min, max y-value
        start_x = 0
        start_x_counter = 0
        end_x = 0
        end_x_counter = 0
        for k in range(int(cluster_with_points[i].size / 2)):
            if cluster_with_points[i][k, 1] == cluster_start_end_points[0, 1, i]:
                start_x += cluster_with_points[i][k, 0]
                start_x_counter +=1
            if cluster_with_points[i][k, 1] == cluster_start_end_points[1, 1, i]:
                end_x +=cluster_with_points[i][k, 0]
                end_x_counter += 1
        cluster_start_end_points[0, 0, i] = int(start_x / start_x_counter)
        cluster_start_end_points[1, 0, i] = int(end_x / end_x_counter)
    return cluster_start_end_points

# to do: mehrere linien erkenne, if abfrage besser machen
def determine_dashed_lines_cluster(cluster_only_dashed):
    cluster_dashed_lines = []
    for i in range(len(cluster_only_dashed)):
        if i == 0:
            cluster_dashed_lines.append(cluster_only_dashed[0])
        else:
            a = cluster_dashed_lines[0]
            b = cluster_only_dashed[i]
            np.concatenate((a, b))
            cluster_dashed_lines[0] = a
    print(len(cluster_dashed_lines))
    return cluster_dashed_lines

def determine_cluster_with_dashed_lines(cluster_with_points, number_of_cluster, cluster_start_end_points):
    cluster_only_solid_lines = []
    cluster_only_dashed_lines = []
    x_diff_treshold = 35
    y_diff_treshold = 350
    for i in range(number_of_cluster):
        is_dashed = False
        for k in range(number_of_cluster):
            # x difference
            x_diff_start_end = abs(cluster_start_end_points[0,0,i] - cluster_start_end_points[1,0,k])
            x_diff_end_start = abs(cluster_start_end_points[1,0,i] - cluster_start_end_points[0,0,k])
            # y difference
            y_diff_start_end = abs(cluster_start_end_points[0,1,i] - cluster_start_end_points[1,1,k])
            y_diff_end_start = abs(cluster_start_end_points[1,1,i] - cluster_start_end_points[0,1,k])
            if (x_diff_start_end < x_diff_treshold and y_diff_start_end < y_diff_treshold):
                is_dashed = True
            if (x_diff_end_start < x_diff_treshold and y_diff_end_start < y_diff_treshold):
                is_dashed = True
        if is_dashed == False:
            cluster_only_solid_lines.append(cluster_with_points[i])
        elif is_dashed == True:
            cluster_only_dashed_lines.append(cluster_with_points[i])

    # append dashed line cluster to solid line cluster
    if not len(cluster_only_dashed_lines) == 0:
        cluster_dashed_lines = []
        # FEHLER, cluster_only_dashed_lines[0] ist anscheinend kein np.array
        cluster_dashed_lines.append(cluster_only_dashed_lines[0])
        for i in range(1, len(cluster_only_dashed_lines), 1):
            a = cluster_dashed_lines[0]
            b = cluster_only_dashed_lines[i]
            cluster_dashed_lines[0] = np.concatenate((a, b))

        cluster_only_solid_lines.append(cluster_dashed_lines[0])

        return cluster_only_solid_lines

    # dashed_lines_cluster = determine_dashed_lines_cluster(cluster_only_dashed_lines)
    # cluster_with_dashed_lines = cluster_without_dashed_lines.append(dashed_lines_cluster)
    return cluster_only_solid_lines

         

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
    cluster_with_points = []

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
                # find min und max y-value
                if int(fields[i]) < cluster_y_value_min:
                    cluster_y_value_min = int(fields[i])
                elif int(fields[i]) > cluster_y_value_max:
                    cluster_y_value_max = int(fields[i])

        # convert string to int
        points = points.astype(int)
        cluster_with_points.append(points)
        
        # draw points on canvas
        draw_circles(points)

        # build function string for message (polydegree, start point, end point)
        function_string = "hello"
        message = message + function_string



        # # determine function of curves
        # function_points = cv2.approxPolyDP(points, 2, False)
        # cv2.drawContours(canvas, function_points, -1, (0, 0, 255), 5)

        # function_points_sorted = sort_function_points(function_points)
        # function = build_function(function_points_sorted)

        # # draw function
        # for i in range(0, 1500, 2):
        #     x = np.polyval(function[0], i)
        #     y = np.polyval(function[1], i)
        #     cv2.circle(canvas, (int(x), int(y)), 1, (0, 255, 0), -1)


    # find start and end points of cluster
    # cluster_with_dashed_lines = determine_cluster_with_dashed_lines(cluster_with_points)
    cluster_start_end_points = determine_cluster_start_end_points(cluster_with_points, number_of_cluster)
    for b in range(number_of_cluster):
        # start point
        cv2.circle(canvas, (int(cluster_start_end_points[0, 0, b]), int(cluster_start_end_points[0, 1, b])), 5, (255, 0, 0), -1)
        # end point
        cv2.circle(canvas, (int(cluster_start_end_points[1, 0, b]), int(cluster_start_end_points[1, 1, b])), 5, (255, 0, 0), -1)

    # find cluster with dashed lines
    cluster_with_dashed_lines = determine_cluster_with_dashed_lines(cluster_with_points, number_of_cluster, cluster_start_end_points)
    
    # find function for cluster
    for c in range(int(len(cluster_with_dashed_lines))):
        # determine function of curves
        function_points = cv2.approxPolyDP(cluster_with_dashed_lines[c], 2, False)
        cv2.drawContours(canvas, function_points, -1, (0, 0, 255), 5)

        function_points_sorted = sort_function_points(function_points)
        function = build_function(function_points_sorted)

        # draw function
        for i in range(0, 1500, 2):                             # to-do: set proper end point
            x = np.polyval(function[0], i)
            y = np.polyval(function[1], i)
            cv2.circle(canvas, (int(x), int(y)), 1, (0, 255, 0), -1)
    
    # # finc function for dashed lines cluster
    # for c in range(int(len(test))):
    #     # determine function of curves
    #     function_points = cv2.approxPolyDP(test[c], 2, False)
    #     cv2.drawContours(canvas, function_points, -1, (0, 0, 255), 5)

    #     function_points_sorted = sort_function_points(function_points)
    #     function = build_function(function_points_sorted)

    #     # draw function
    #     for i in range(0, 1500, 2):                             # to-do: set proper end point
    #         x = np.polyval(function[0], i)
    #         y = np.polyval(function[1], i)
    #         cv2.circle(canvas, (int(x), int(y)), 1, (0, 0, 255), -1)


    # talker(str(poly_degree) + ";" + message)
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
