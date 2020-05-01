#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from laneregression.msg import functionData, functionArray
import numpy as np
import cv2
import random
import math

__author__ = 'Yannik Motzet'
__copyright__ = 'Copyright 2020, ZF DHBW Innovation Lab'
__version__ = '0.1.0'
__email__ = 'yannik.motzet@outlook.com'
__status__ = 'in development'


picHeight = 960
picWidth = 1280
truck_pos_x = int(picWidth / 2)
poly_degree = 2
canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
data_function_collected = ""


def draw_circles(points):
    r, g, b = random.randint(0, 255), random.randint(
        0, 255), random.randint(0, 255)
    # r, g, b = 0, 0, 0
    for i in range(int(points.size / 2)):
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1, (r, g, b), -1)


def calcluate_distance_between_points(x1, y1, x2, y2):
    x_diff = abs(x2 - x1)
    y_diff = abs(y2 - y1)
    return math.sqrt(x_diff**2 + y_diff**2)

# returns the start and end points of all clusters


def determine_cluster_start_end_points(cluster_with_points, number_of_cluster):
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
                start_x_counter += 1
            if cluster_with_points[i][k, 1] == cluster_start_end_points[1, 1, i]:
                end_x += cluster_with_points[i][k, 0]
                end_x_counter += 1
        cluster_start_end_points[0, 0, i] = int(start_x / start_x_counter)
        cluster_start_end_points[1, 0, i] = int(end_x / end_x_counter)
    return cluster_start_end_points


# takes unsorted cluster with cluster of dashed lines and looks for related dashed lines
# ACHTUNG! momentan werden alle gestrichelten cluster zu einer gestrichelten Linie zusammengefasst
# eine unterscheidung, ob es mehere gestrichelte linien gibt, erfolgt nicht
# TODO mehrere linien erkenne, if abfrage besser machen
# TODO Fehlererkennung, macht Cluster Sinn oder falsches Objekt uebermittelt?
def determine_dashed_lines_cluster(cluster_only_dashed):
    if not len(cluster_only_dashed) == 0:
        cluster_dashed_lines = []
        cluster_dashed_lines.append(cluster_only_dashed[0])
        for i in range(1, len(cluster_only_dashed), 1):
            a = cluster_dashed_lines[0]
            b = cluster_only_dashed[i]
            cluster_dashed_lines[0] = np.concatenate((a, b))
        return cluster_dashed_lines[0]
    else:
        return None


# takes all cluster and returns:
# 1) cluster with solid and dashed lines (datatype of return is list with np.array as items)
# 2) metadata (return datatype is list with integer items, the integer correspond to the type of the lines)
def determine_cluster_with_solid_and_dashed_lines(cluster_with_points, number_of_cluster, cluster_start_end_points):

    # values for meta data: 0 == undefined,  1 == solid, 2 == dashed
    META_UNDEFINED = 0
    META_SOLID = 1
    META_DASHED = 2

    cluster_only_solid_lines = []
    cluster_only_dashed_lines = []
    cluster_meta_data = []
    x_diff_treshold = 35
    y_diff_treshold = 350

    cluster = cluster_with_points
    start_end_points = cluster_start_end_points
    meta = []
    for i in range(len(cluster)):
        meta.append(META_UNDEFINED)
    k = 0

    while (k < len(cluster)):
        i = k + 1
        while (i <= len(cluster) - 1):
            # x difference
            x_diff_start_end = abs(
                start_end_points[0, 0, k] - start_end_points[1, 0, i])
            x_diff_end_start = abs(
                start_end_points[1, 0, k] - start_end_points[0, 0, i])
            # y difference
            y_diff_start_end = abs(
                start_end_points[0, 1, k] - start_end_points[1, 1, i])
            y_diff_end_start = abs(
                start_end_points[1, 1, k] - start_end_points[0, 1, i])

            # compare start and end point
            if (x_diff_start_end < x_diff_treshold and y_diff_start_end < y_diff_treshold):
                # append cluster and adjust start end points
                cluster[k] = np.append(cluster[k], cluster[i], 0)
                del cluster[i]
                # overwrite start point of k with start point of i, delete i
                start_end_points[0, 0, k] = start_end_points[0, 0, i]
                start_end_points[0, 1, k] = start_end_points[0, 1, i]
                start_end_points = np.delete(start_end_points, i, 2)
                meta[k] = META_DASHED
                i = k + 1

            # compare end and start point
            elif (x_diff_end_start < x_diff_treshold and y_diff_end_start < y_diff_treshold):
                # append cluster and adjust start end points
                cluster[k] = np.append(cluster[k], cluster[i], 0)
                del cluster[i]
                # overwrite end point of k with end point of i, delete i
                start_end_points[1, 0, k] = start_end_points[1, 0, i]
                start_end_points[1, 1, k] = start_end_points[1, 1, i]
                start_end_points = np.delete(start_end_points, i, 2)
                meta[k] = META_DASHED
                i = k + 1
            else:
                i = i + 1

        if meta[k] == META_UNDEFINED:
            meta[k] = META_SOLID
        k = k + 1

    # check that meta list has same length like cluster list
    del meta[len(cluster):]

    return cluster, meta


# TODO: sort clockwise (https://stackoverflow.com/questions/51074984/sorting-according-to-clockwise-point-coordinates/51075469, https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/)
# sorts function points successively by looking for nearest neighbor (O(n^2))
def sort_function_points_improved(points):

    treshold_distance = 50

    number_of_points = int(points.size / 2)
    function_points_unsorted = np.reshape(points, (number_of_points, 2))

    # find anchor point, anchor point = lowest point (max y)
    index_anchor_point = 0
    value_anchor_point = 0
    for i in range(number_of_points):
        if function_points_unsorted[i, 1] > value_anchor_point:
            index_anchor_point = i
            value_anchor_point = function_points_unsorted[i, 1]

    # insert anchor point in new np array, delete from old array
    function_points_sorted = np.array(
        [[function_points_unsorted[index_anchor_point, 0], function_points_unsorted[index_anchor_point, 1]]])
    function_points_unsorted = np.delete(
        function_points_unsorted, index_anchor_point, 0)

    # sort points
    # for focused_point_index in range(0, number_of_points - 2):  # anchor point is already processed, leave last point
    while (int(function_points_unsorted.size / 2) > 2):

        # start value is diagonal of picture
        smallest_distance = math.sqrt(picHeight**2 + picWidth**2)
        index_point_smallest_distance = - 1

        focused_point_index = int(function_points_sorted.size / 2) - 1
        x1 = function_points_sorted[focused_point_index, 0]
        y1 = function_points_sorted[focused_point_index, 1]

        for k in range(int(function_points_unsorted.size / 2)):
            x2 = function_points_unsorted[k, 0]
            y2 = function_points_unsorted[k, 1]

            # looks for neighbour with smallest distance
            if calcluate_distance_between_points(x1, y1, x2, y2) < smallest_distance:
                smallest_distance = calcluate_distance_between_points(
                    x1, y1, x2, y2)
                index_point_smallest_distance = k

        # treshold for distance
        if smallest_distance > treshold_distance:
            function_points_sorted = np.append(
                function_points_sorted, function_points_unsorted[index_point_smallest_distance, :].reshape(1, 2), axis=0)
            function_points_unsorted = np.delete(
                function_points_unsorted, index_point_smallest_distance, 0)
        else:
            function_points_unsorted = np.delete(
                function_points_unsorted, index_point_smallest_distance, 0)

    # add last point
    function_points_sorted = np.append(
        function_points_sorted, function_points_unsorted[0, :].reshape(1, 2), axis=0)

    # # for debugging. show picture with points and order number
    # print(number_of_points)
    # print(int(function_points_sorted.size /2))
    # test = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    # for n in range (int(function_points_sorted.size /2)):
    #     cv2.putText(test, str(n), (function_points_sorted[n, 0] + int(random.randint(0, 10)),function_points_sorted[n, 1] + int(random.randint(0, 10))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0)
    #     cv2.circle(test, (function_points_sorted[n, 0],function_points_sorted[n, 1]), 2, (0, 255, 0), -1)

    # cv2.imshow("sort", sort)
    # cv2.imwrite('sort.png', sort)
    # cv2.waitKey(10000)
    # cv2.destroyAllWindows()

    return function_points_sorted


# sorts the function points
def sort_function_points(function_points):
    number_of_points = int(function_points.size / 2)
    function_points = np.reshape(function_points, (number_of_points, 2))
    # https://stackoverflow.com/a/2828371
    # function_points = np.sort(function_points.view('i4,i4'), order=['f1','f0'], axis=0).view(np.int)
    function_points.view('i4,i4').sort(order=['f1', 'f0'], axis=0)
    return function_points

# using the sorted function points to build a function with x(t) and y(t)


def build_function(function_points_sorted):
    number_of_points = int(function_points_sorted.size / 2)
    value_table = np.zeros((number_of_points, 3))
    value_table[:, 1:] = function_points_sorted
    value_table[0, 0] = 0
    for i in range(1, number_of_points):
        value_table[i, 0] = value_table[i-1, 0] + calcluate_distance_between_points(
            value_table[i, 1], value_table[i, 2], value_table[i-1, 1], value_table[i-1, 2])
    # regression for x
    x_function = np.polyfit(value_table[:, 0], value_table[:, 1], poly_degree)
    # regression for y
    y_function = np.polyfit(value_table[:, 0], value_table[:, 2], poly_degree)
    return [x_function, y_function]


def get_order_of_lines(functions):
    lines_pos_x = []
    left_lines_index = []
    right_lines_index = []

    for i in range(0, len(functions), 2):
        a = functions[i][0]
        b = functions[i][1]
        c = functions[i][2]
        d = functions[i+1][0]
        e = functions[i+1][1]
        f = functions[i+1][2] - truck_pos_x

        # determine value of t at intersection with lower picture border
        discriminant = math.sqrt(e**2 - (4 * d * f))

        t_1 = (-e + discriminant) / (2*d)
        t_2 = (-e - discriminant) / (2*d)

        # determine x value for t value
        x = a*t_2**2 + b*t_2 + c

        lines_pos_x.append(x)
    
    #sort lines_pos_x and get sorted index back
    index_sorted = sorted(range(len(lines_pos_x)), key=lines_pos_x.__getitem__)

    # check if line is left or right of truck position
    for i in range(len(index_sorted)):
        index = index_sorted[i]
        if lines_pos_x[index] < truck_pos_x:
            left_lines_index.append(index)
        elif lines_pos_x[index] >= truck_pos_x:
            right_lines_index.append(index)

    # reverse left_lines_index (truck_pos is anchor and you cound lines from right to left)
    left_lines_index.reverse()


    return right_lines_index, left_lines_index


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    # clusters
    global canvas
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    cluster_with_points = []
    all_functions = []
    all_meta = []

    cluster = data.data.split(";")
    number_of_cluster = len(cluster) - 1

    # draw points of raw cluster on canvas
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
        # append points of current cluster to global list
        cluster_with_points.append(points)

        # draw point cloud of current raw cluster on canvas
        draw_circles(points)

    # find start and end points of cluster
    cluster_start_end_points = determine_cluster_start_end_points(
        cluster_with_points, number_of_cluster)
    # draw start and end point of each cluster
    for b in range(number_of_cluster):
        # start point
        cv2.circle(canvas, (int(cluster_start_end_points[0, 0, b]), int(
            cluster_start_end_points[0, 1, b])), 5, (255, 0, 0), -1)
        # end point
        cv2.circle(canvas, (int(cluster_start_end_points[1, 0, b]), int(
            cluster_start_end_points[1, 1, b])), 5, (255, 0, 0), -1)

    # find cluster with solid lines + dashed lines (line cluster)
    cluster_with_solid_and_dashed_lines, cluster_meta_data = determine_cluster_with_solid_and_dashed_lines(
        cluster_with_points, number_of_cluster, cluster_start_end_points)

    for l in cluster_meta_data:
        all_meta.append(l)

    # draw new cluster with solid lines + dashed lines on canvas
    # for u in range(int(len(cluster_with_solid_and_dashed_lines))):
    #     draw_circles(cluster_with_solid_and_dashed_lines[u])

    # find function for each line
    for c in range(int(len(cluster_with_solid_and_dashed_lines))):
        # determine some points of cluster with Douglas-Peucker algorithm
        function_points = cv2.approxPolyDP(
            cluster_with_solid_and_dashed_lines[c], 2, False)
        cv2.drawContours(canvas, function_points, -1, (0, 0, 255), 5)
        if len(function_points) < 3:
            print(
                "warning: function of a line might be faulty due to too less function points")

        # put the points in order
        # function_points_sorted = sort_function_points(function_points)
        function_points_sorted = sort_function_points_improved(function_points)

        # determine function by getting x(t) and y(t), function is a list, elements are nparray, function[0] contains parameters for x(t), function[1] for y(t)
        function = build_function(function_points_sorted)
        for m in function:
            all_functions.append(m)

        # draw function
        # TODO: set proper end point
        for i in range(-500, 1500, 2):
            x = np.polyval(function[0], i)
            y = np.polyval(function[1], i)
            cv2.circle(canvas, (int(x), int(y)), 1, (0, 255, 0), -1)

        # draw position of truck
        cv2.line(canvas, (int(picWidth/2), picHeight),
                 (int(picWidth/2), picHeight - 20), (0, 0, 0), 5)


    # sort lines
    index_right, index_left = get_order_of_lines(all_functions)

    # len(order_of_lines), len(meta), int(len(all_functions)/2) should be identical
    if len(all_meta) != int(len(all_functions)/2): print("warning: number of functions does not correspond to number of meta")
    
    # ROS topic
    function_array = functionArray()
    # left lines
    for p in range(len(index_left)):
        index = index_left[p]
        function_data = functionData()
        function_data.position = (p + 1)*(-1)
        function_data.meta = all_meta[index]
        function_data.a = all_functions[index*2][0]
        function_data.d = all_functions[index*2+1][0]
        function_data.b = all_functions[index*2][1]
        function_data.e = all_functions[index*2+1][1]
        function_data.c = all_functions[index*2][2]
        function_data.f = all_functions[index*2+1][2]

        function_array.functions.append(function_data)

    # right lines
    for p in range(len(index_right)):
        index = index_right[p]
        function_data = functionData()
        function_data.position = p
        function_data.meta = all_meta[index]
        function_data.a = all_functions[index*2][0]
        function_data.d = all_functions[index*2+1][0]
        function_data.b = all_functions[index*2][1]
        function_data.e = all_functions[index*2+1][1]
        function_data.c = all_functions[index*2][2]
        function_data.f = all_functions[index*2+1][2]

        function_array.functions.append(function_data)


    # send a message via ROS topic
    talker(function_array.functions)

    # display canvas window
    cv2.imshow("points", canvas)
    cv2.imwrite('result.png', canvas)
    cv2.waitKey(2000)
    # cv2.destroyAllWindows()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laneregression', anonymous=True)

    rospy.Subscriber("lanedetection_cluster", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker(data_function):
    pub = rospy.Publisher('laneregression_functions', functionArray, queue_size=10)
    # rospy.init_node('talker_function', anonymous=True)
    rate = rospy.Rate(10)  # hz
    if not rospy.is_shutdown():
        rospy.loginfo(data_function)
        pub.publish(data_function)
        rate.sleep()


if __name__ == '__main__':
    listener()
