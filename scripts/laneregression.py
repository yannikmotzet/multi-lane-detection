#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from lane_keeping_assist.msg import functionData, functionArray, cluster_data
import numpy as np
import cv2
import random
import math
import time

__author__ = 'Yannik Motzet'
__copyright__ = 'Copyright 2020, ZF-DHBW Innovation Lab'
__version__ = '0.9.9'
__email__ = 'yannik.motzet@outlook.com'
__status__ = 'in development'


# picture resoultion
# PICTURE_HEIGHT = 960
PICTURE_HEIGHT = 660
PICTURE_WIDTH = 1280
# width of track
# TRACK_WIDTH = 143
TRACK_WIDTH = 100
# degree for polynomial regression
POLY_DEGREE = 2
# reduce points of lanedetection with polydp (Douglas-Ramer)
REDUCE_RAW_POINTS = False
# minimum length a cluster should have (from start to end pont)
MIN_CLUSTER_LENGTH = 50
# cluster which start under this value will be deleted
MIN_CLUSTER_HEIGHT = 300
# distance of lane segments (important for cluster_lane_segments method)
DASHED_X_DISTANCE = 80
DASHED_Y_DISTANCE = 100
# parameters for segments_lie_on_a_line method
ANGLE_DIFFERENCE = 10
SEGMENT_X_DIFFERENCE = 20
# minimum distance function points should have after sorting
SORT_POINT_DISTANCE = 20
# display window
DISPLAY_CANVAS = True
# print execution time of different code blocks
DISPLAY_TIME = False


TRUCK_POS_X = int(PICTURE_WIDTH / 2)
canvas = 255 * np.ones(shape=[PICTURE_HEIGHT, PICTURE_WIDTH, 3], dtype=np.uint8)


def draw_circles(points):
    b, g, r = random.randint(0, 255), random.randint(
        0, 255), random.randint(0, 255)
    # b, g, r = 0, 0, 0
    for i in range(int(points.size / 2)):
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1, (b, g, r), -1)


def calculuate_distance_between_points(x1, y1, x2, y2):
    x_diff = abs(x2 - x1)
    y_diff = abs(y2 - y1)
    return math.sqrt(x_diff**2 + y_diff**2)


# returns the start and end points of all clusters
def determine_cluster_start_end_points(cluster_with_points, number_of_cluster):
    cluster_start_end_points = np.zeros((2, 2, number_of_cluster))
    # determine start and end points
    for i in range(number_of_cluster):
        start_y = PICTURE_HEIGHT
        end_y = 0
        # determine min, max y-value
        for k in range(int(cluster_with_points[i].size / 2)):
            if (cluster_with_points[i])[k, 1] < start_y:
                start_y = (cluster_with_points[i])[k, 1]
            if (cluster_with_points[i])[k, 1] > end_y:
                end_y = (cluster_with_points[i])[k, 1]
        cluster_start_end_points[0, 1, i] = start_y
        cluster_start_end_points[1, 1, i] = end_y

        # determine x-value of min, max y-value (it is possible that there are several points with same y-value, we want the middele point)
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


# function that calculates area of ractangle depending on the side lengths
def calculate_area_rectangle(x_length, y_length):
    return abs(x_length * y_length)


# finds intersection point of two lines
# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
def determine_line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
    err = False

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y


# check if point (x, y) lies in rectangle (p1 (left top), p4 (bottom right))
def check_if_point_in_rectangle(x1, y1, x4, y4, x, y):
    return x1*0.9-10 <= x <= x4*1.1+10 and y1*0.9-10 <= y <= y4*1.1+10


# calculates angele of vector
def calculate_angle_between_points(start_x, start_y, end_x, end_y):
    radian = np.arctan(abs(start_x - end_x) / abs(start_y - end_y))
    return (radian  / (2 * math.pi) * 360)


# check if two lane segments lie on the same line
def segments_lie_on_a_line(start_1, end_1, start_2, end_2):
    angle1 = calculate_angle_between_points(start_1[0], start_1[1],end_1[0], end_1[1])
    angle2 = calculate_angle_between_points(start_2[0], start_2[1],end_2[0], end_2[1])
    
    global ANGLE_DIFFERENCE, SEGMENT_X_DIFFERENCE
    
    if abs(angle1 - angle2) <= ANGLE_DIFFERENCE:
        # build function (y = mx + c) of 2nd line (https://www.arndt-bruenner.de/mathe/9/geradedurchzweipunkte.htm)
        if end_2[0] != start_2[0]:
            # m = (y2-y1)/(x2-x1)
            m_2 =  (end_2[1] - start_2[1]) / (end_2[0] - start_2[0])
            # c = (x2*y1 - x1*y2) / (x2-x1)
            c_2 = (end_2[0] * start_2[1] - start_2[0] * end_2[1]) / (end_2[0] - start_2[0])
            # get x value of 2nd function where y = y-value of end point of 1st function (x = (y-value - c) / m)
            x_2 = (start_1[1] - c_2) / m_2
            # segments are on same line if difference of their x-values are smaller than a certain value
            if abs(start_1[0] - x_2) <= SEGMENT_X_DIFFERENCE:
                return True
        else:
            # m = (y2-y1)/(x2-x1)
            m_2 =  (end_2[1] - start_2[1]) / (end_2[0] - (start_2[0]+1))
            # c = (x2*y1 - x1*y2) / (x2-x1)
            c_2 = (end_2[0] * start_2[1] - (start_2[0]+1) * end_2[1]) / (end_2[0] - (start_2[0]+1))
            # get x value of 2nd function where y = y-value of end point of 1st function (x = (y-value - c) / m)
            x_2 = (start_1[1] - c_2) / m_2
            # segments are on same line if difference of their x-values are smaller than a certain value
            if abs(start_1[0] - x_2) <= SEGMENT_X_DIFFERENCE:
                return True

    return False


# takes all cluster and returns:
# 1) cluster with solid and dashed lines (datatype of return is list with np.array as items)
# 2) metadata (return datatype is list with integer items, the integer correspond to the type of the lines)
def cluster_lane_segments(cluster_with_points, cluster_start_end_points):
    global DASHED_X_DISTANCE
    global DASHED_Y_DISTANCE

    # values for meta data: 0 == undefined,  1 == solid, 2 == dashed
    META_UNDEFINED = 0
    META_SOLID = 1
    META_DASHED = 2
    
    cluster = cluster_with_points
    start_end_points = cluster_start_end_points
    meta = []

    # fill meta list
    for i in range(len(cluster)):
        meta.append(META_UNDEFINED)

    n = len(cluster)
    comparison_matrix = np.full((n, n), False)
    single_cluster_list = np.full((n), False)

    # build comparison matrix
    for i in range(n):
        for j in range(n):
            if i != j:
                # compare intersection
                line1 = ((start_end_points[0, 0, i] ,start_end_points[0, 1, i]), (start_end_points[1, 0, i] ,start_end_points[1, 1, i]))
                line2 = ((start_end_points[0, 0, j], start_end_points[0, 1, j]), (start_end_points[1, 0, j], start_end_points[1, 1, j]))
                try:
                    intersection_x, intersection_y = determine_line_intersection(line1, line2)
                    has_intersection = True
                except:
                    has_intersection = False

                # draw intersection
                # if DISPLAY_CANVAS:
                #     r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
                #     cv2.circle(canvas, (int(intersection_x), int(intersection_y)), 5, (r, g, b), -1)

                if has_intersection:
                    # sement i is under segment j
                    if start_end_points[0, 1, i] > start_end_points[0, 1, j] and start_end_points[1, 1, i] > start_end_points[1, 1, j]:
                        rectangle_x1 = int(min(start_end_points[0, 0, i], start_end_points[1, 0, j]))
                        rectangle_y1 = int(min(start_end_points[0, 1, i], start_end_points[1, 1, j]))
                        rectangle_x2 = int(max(start_end_points[0, 0, i], start_end_points[1, 0, j]))
                        rectangle_y2 = rectangle_y1
                        rectangle_x3 = rectangle_x1
                        rectangle_y3 = int(max(start_end_points[0, 1, i], start_end_points[1, 1, j]))
                        rectangle_x4 = rectangle_x2
                        rectangle_y4 = rectangle_y3

                        # draw rectangle
                        # if DISPLAY_CANVAS:
                        #     cv2.rectangle(canvas, (int(rectangle_x2), int(rectangle_y2)), (int(rectangle_x3), int(rectangle_y3)), (r, g, b), 1)
                        
                        area = calculate_area_rectangle(abs(rectangle_x2 - rectangle_x1), abs(rectangle_y3 - rectangle_y1))
                        # area should be smaller than a certain area, intersection should be in area
                        if area <= (DASHED_X_DISTANCE * DASHED_Y_DISTANCE) and check_if_point_in_rectangle(rectangle_x1, rectangle_y1, rectangle_x4, rectangle_y4, intersection_x, intersection_y):
                            comparison_matrix[i, j] = True
                            single_cluster_list[i] = True
                            continue
                        
                    # sement i is above segment j
                    elif start_end_points[0, 1, j] > start_end_points[0, 1, i] and start_end_points[1, 1, j] > start_end_points[1, 1, i]:
                        rectangle_x1 = int(min(start_end_points[1, 0, i], start_end_points[0, 0, j]))
                        rectangle_y1 = int(min(start_end_points[1, 1, i], start_end_points[0, 1, j]))
                        rectangle_x2 = int(max(start_end_points[1, 0, i], start_end_points[0, 0, j]))
                        rectangle_y2 = rectangle_y1
                        rectangle_x3 = rectangle_x1
                        rectangle_y3 = int(max(start_end_points[1, 1, i], start_end_points[0, 1, j]))
                        rectangle_x4 = rectangle_x2
                        rectangle_y4 = rectangle_y3

                        # draw rectangle
                        # if DISPLAY_CANVAS:
                        #     cv2.rectangle(canvas, (int(rectangle_x2), int(rectangle_y2)), (int(rectangle_x3), int(rectangle_y3)), (r, g, b), 1)
                        
                        area = calculate_area_rectangle(abs(rectangle_x2 - rectangle_x1), abs(rectangle_y3 - rectangle_y1))
                        # area should be smaller than a certain area, intersection should be in area
                        if area <= (DASHED_X_DISTANCE * DASHED_Y_DISTANCE) and check_if_point_in_rectangle(rectangle_x1, rectangle_y1, rectangle_x4, rectangle_y4, intersection_x, intersection_y):
                            comparison_matrix[i, j] = True
                            single_cluster_list[i] = True
                            continue
                        
                        # check for same angle and distance
                        else: 
                            if segments_lie_on_a_line(start_end_points[0, :, j], start_end_points[1, :, j], start_end_points[0, :, i], start_end_points[1, :, i]):
                                comparison_matrix[i, j] = True
                                single_cluster_list[i] = True

                # check for same angle and distance
                # sement i is under segment j
                if start_end_points[0, 1, i] > start_end_points[0, 1, j] and start_end_points[1, 1, i] > start_end_points[1, 1, j]:
                    if segments_lie_on_a_line(start_end_points[0, :, i], start_end_points[1, :, i], start_end_points[0, :, j], start_end_points[1, :, j]):
                        comparison_matrix[i, j] = True
                        single_cluster_list[i] = True
                # sement i is above segment j
                elif start_end_points[0, 1, j] > start_end_points[0, 1, i] and start_end_points[1, 1, j] > start_end_points[1, 1, i]:
                    if segments_lie_on_a_line(start_end_points[0, :, j], start_end_points[1, :, j], start_end_points[0, :, i], start_end_points[1, :, i]):
                        comparison_matrix[i, j] = True
                        single_cluster_list[i] = True


    # find transitive hull with Warshall algorithm (https://de.wikipedia.org/wiki/Algorithmus_von_Floyd_und_Warshall)
    for k in range(n):
        for i in range(n):
            if comparison_matrix[i, k]:
                for j in range(n):
                    if comparison_matrix[k, j]:
                        comparison_matrix[i, j] = True
    
    # build new cluster
    new_cluster = []
    already_appended = np.full((n), False)

    for i in range(n):
        if single_cluster_list[i]:
            single_cluster_list[i] = False
            new_cluster.append(cluster[i])
            for j in range(i, n):
                if i != j:
                    if comparison_matrix[i, j]:
                        new_cluster[len(new_cluster) - 1] = np.append(new_cluster[len(new_cluster) - 1], cluster[j], 0)
                        meta[i] = META_DASHED
                        single_cluster_list[j] = False
                        already_appended[j] = True
        elif already_appended[i] == False:
            new_cluster.append(cluster[i])
            meta[i] = META_SOLID


    # check that meta list has same length like cluster list
    del meta[len(new_cluster):]

    return new_cluster, meta


# NOT IN USE!
# sorts the function points by y-value
def sort_function_points(function_points):
    number_of_points = int(function_points.size / 2)
    function_points = np.reshape(function_points, (number_of_points, 2))
    # https://stackoverflow.com/a/2828371
    # function_points = np.sort(function_points.view('i4,i4'), order=['f1','f0'], axis=0).view(np.int)
    function_points.view('i4,i4').sort(order=['f1', 'f0'], axis=0)
    return function_points


# sorts function points successively by looking for nearest neighbor
def sort_function_points_improved(points):

    global SORT_POINT_DISTANCE

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
        smallest_distance = math.sqrt(PICTURE_HEIGHT**2 + PICTURE_WIDTH**2)
        index_point_smallest_distance = - 1

        focused_point_index = int(function_points_sorted.size / 2) - 1
        x1 = function_points_sorted[focused_point_index, 0]
        y1 = function_points_sorted[focused_point_index, 1]

        for k in range(int(function_points_unsorted.size / 2)):
            x2 = function_points_unsorted[k, 0]
            y2 = function_points_unsorted[k, 1]

            # looks for neighbour with smallest distance
            if calculuate_distance_between_points(x1, y1, x2, y2) < smallest_distance:
                smallest_distance = calculuate_distance_between_points(
                    x1, y1, x2, y2)
                index_point_smallest_distance = k

        # threshold for distance
        if smallest_distance > SORT_POINT_DISTANCE:
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
    # test = 255 * np.ones(shape=[PICTURE_HEIGHT, PICTURE_WIDTH, 3], dtype=np.uint8)
    # for n in range (int(function_points_sorted.size /2)):
    #     cv2.putText(test, str(n), (function_points_sorted[n, 0] + int(random.randint(0, 10)),function_points_sorted[n, 1] + int(random.randint(0, 10))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0)
    #     cv2.circle(test, (function_points_sorted[n, 0],function_points_sorted[n, 1]), 2, (0, 255, 0), -1)

    # cv2.imshow("sort", sort)
    # cv2.imwrite('sort.png', sort)
    # cv2.waitKey(10000)
    # cv2.destroyAllWindows()

    return function_points_sorted


# using the sorted function points to build a function with x(t) and y(t)
def build_function(function_points_sorted):
    number_of_points = int(function_points_sorted.size / 2)
    value_table = np.zeros((number_of_points, 3))
    value_table[:, 1:] = function_points_sorted
    value_table[0, 0] = 0
    for i in range(1, number_of_points):
        value_table[i, 0] = value_table[i-1, 0] + calculuate_distance_between_points(
            value_table[i, 1], value_table[i, 2], value_table[i-1, 1], value_table[i-1, 2])
    # regression for x
    x_function = np.polyfit(value_table[:, 0], value_table[:, 1], POLY_DEGREE)
    # regression for y
    y_function = np.polyfit(value_table[:, 0], value_table[:, 2], POLY_DEGREE)
    return [x_function, y_function]


# determination of arrangement of the lines, returns position index for each line (0: right border line, 1: left border line)
def get_order_of_lines(functions):
    lines_pos_x = []
    left_lines_index = []
    right_lines_index = []

    for i in range(0, len(functions), 2):
        # x(t)
        a = functions[i][0]
        b = functions[i][1]
        c = functions[i][2]

        # y(t)
        d = functions[i+1][0]
        e = functions[i+1][1]
        f = functions[i+1][2] - (PICTURE_HEIGHT)     # substract PICTURE_HEIGHT to find intersection with lower picture border, not the zeropoints

        # determine value of t at intersection with lower picture border -> quadratic formula
        discriminant = math.sqrt((e**2) - (4 * d * f))

        t_1 = (-e + discriminant) / (2*d)
        t_2 = (-e - discriminant) / (2*d)

        # determine x value for t value
        x_1 = (a*(t_1**2)) + (b*t_1) + c
        x_2 = (a*(t_2**2)) + (b*t_2) + c

        lines_pos_x.append(int(x_2))
    
    #sort lines_pos_x and get sorted index back
    index_sorted = sorted(range(len(lines_pos_x)), key=lines_pos_x.__getitem__)

    # check if line is left or right of truck position
    for i in range(len(index_sorted)):
        index = index_sorted[i]
        if lines_pos_x[index] < TRUCK_POS_X:
            left_lines_index.append(index)
        elif lines_pos_x[index] >= TRUCK_POS_X:
            right_lines_index.append(index)

    # reverse left_lines_index (truck_pos is anchor and you cound lines from right to left)
    left_lines_index.reverse()

    return right_lines_index, left_lines_index, lines_pos_x


######################################################################################################################
######################################################################################################################
######################################################################################################################
######################################################################################################################


# callback function
####################################################
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    global canvas
    global REDUCE_RAW_POINTS
    canvas = 255 * np.ones(shape=[PICTURE_HEIGHT, PICTURE_WIDTH, 3], dtype=np.uint8)
    cluster_with_points = []
    all_functions = []
    all_meta = []

    number_of_cluster = len(data.size)

    # write points of ROS message to list
    ####################################################
    # data.size has information about length of the single clusters
    # data.points contains all points of cluster
    index = 0
    for n in range(len(data.size)):
        
        points = np.zeros((data.size[n], 2))
        for i in range(data.size[n]):
            points[i, 0] = data.points[index + i].x
            points[i, 1] = data.points[index + i].y
        index = index + data.size[n]

        if REDUCE_RAW_POINTS == True:
            if points.shape[0] > 100:
                reduced_points = cv2.approxPolyDP(points, 2, False)
                print("reduced number of points from " + str(points.shape[0]) + " to " + str(reduced_points.shape[0]))
                points = reduced_points.reshape(reduced_points.shape[0],2)

        points = points.astype(int)

        cluster_with_points.append(points)

        # draw raw points
        # if display_output:
        #     draw_circles(points)

    
    # find dashed/solid lines
    ####################################################
    # find start and end points of cluster
    time0 = time.time()
    cluster_start_end_points = determine_cluster_start_end_points(
        cluster_with_points, number_of_cluster)
    time1 = time.time()
    if DISPLAY_TIME:
        print("time0 " + str(time1 - time0))

    # draw start and end point of each cluster
    if DISPLAY_CANVAS:
        for b in range(number_of_cluster):
            # start point
            cv2.circle(canvas, (int(cluster_start_end_points[0, 0, b]), int(
                cluster_start_end_points[0, 1, b])), 5, (255, 0, 0), -1)
            # end point
            cv2.circle(canvas, (int(cluster_start_end_points[1, 0, b]), int(
                cluster_start_end_points[1, 1, b])), 5, (255, 0, 0), -1)

    
    # find cluster with solid lines + dashed lines (line cluster)
    # cluster_with_solid_and_dashed_lines, cluster_meta_data = determine_cluster_with_solid_and_dashed_lines(
    #     cluster_with_points, cluster_start_end_points
    time2 = time.time()
    cluster_with_solid_and_dashed_lines, cluster_meta_data = cluster_lane_segments(
        cluster_with_points, cluster_start_end_points)
    time3 = time.time()
    if DISPLAY_TIME:
        print("time1 " + str(time3 - time2))

    # # draw new cluster with solid lines + dashed lines on canvas
    # if display_output:
        # for u in range(int(len(cluster_with_solid_and_dashed_lines))):
        #     draw_circles(cluster_with_solid_and_dashed_lines[u])


    # find function for each line
    ####################################################
    time4 = time.time()
    reduce_points = False
    for c in range(int(len(cluster_with_solid_and_dashed_lines))):
        # reduce points with polydp algorithm?
        if reduce_points == True:
            # reduce points of cluster with Douglas-Peucker algorithm
            function_points = cv2.approxPolyDP(
                cluster_with_solid_and_dashed_lines[c], 2, False)
            # print("reduced number of points from " + str(cluster_with_solid_and_dashed_lines[c].shape[0]) + " to " + str(function_points.shape[0]))
        else:
            function_points = cluster_with_solid_and_dashed_lines[c].reshape(cluster_with_solid_and_dashed_lines[c].shape[0],1,2)
        
        # draw points 
        # if display_output:
            # b, g, r = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
            # cv2.drawContours(canvas, function_points, -1, (b, g, r), 5)
        
        if len(function_points) < 3:
            print("warning: function of a line might be faulty due to too less function points")

        # put the points in order
        # function_points_sorted = sort_function_points(function_points)
        function_points_sorted = sort_function_points_improved(function_points)

        # filter small and high cluster
        global MIN_CLUSTER_LENGTH
        global MIN_CLUSTER_HEIGHT
        cluster_number_points = function_points_sorted.shape[0] - 1
        cluster_line_length = calculuate_distance_between_points(function_points_sorted[0,0], function_points_sorted[0,1], function_points_sorted[cluster_number_points, 0], function_points_sorted[cluster_number_points, 1])
        # delete to small cluster        
        if cluster_line_length < MIN_CLUSTER_LENGTH:
            continue
        # delete cluster which start to high in the picture
        if function_points_sorted[0, 1] < MIN_CLUSTER_HEIGHT:
            continue

        # determine function by getting x(t) and y(t), function is a list, elements are nparray, function[0] contains parameters for x(t), function[1] for y(t)
        function = build_function(function_points_sorted)
        for m in function:
            all_functions.append(m)

        all_meta.append(cluster_meta_data[c])

        # # draw function
        # if display_output:
        #     for i in range(-500, 1500, 2):
        #         x = np.polyval(function[0], i)
        #         y = np.polyval(function[1], i)
        #         cv2.circle(canvas, (int(x), int(y)), 1, (0, 255, 0), -1)

        # draw position of truck
        cv2.line(canvas, (int(PICTURE_WIDTH/2), PICTURE_HEIGHT),
                 (int(PICTURE_WIDTH/2), PICTURE_HEIGHT - 20), (0, 0, 0), 5)
    
    time5 = time.time()
    if DISPLAY_TIME:
        print("time2 " + str(time5 - time4))


    # sort lines by position
    ####################################################
    index_right, index_left, x_pos = get_order_of_lines(all_functions)

    # len(order_of_lines), len(meta), int(len(all_functions)/2) should be identical
    if len(all_meta) != int(len(all_functions)/2): print("warning: number of functions does not correspond to number of meta")
    
    
    # ROS message for storing line information
    # includes koefficents of functions (#x(s) = as^2+bs+c, #y(s) = ds^2+es+f), meta information and position
    ####################################################
    time6 = time.time()
    function_array = functionArray()
    
    # left lines
    for p in range(len(index_left)):
        index = index_left[p]

        function_data = functionData()
        function_data.position = (p + 1)
        function_data.x_position = x_pos[index]
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
        function_data.position = p*(-1)
        function_data.x_position = x_pos[index]
        function_data.meta = all_meta[index]

        function_data.a = all_functions[index*2][0]
        function_data.d = all_functions[index*2+1][0]
        function_data.b = all_functions[index*2][1]
        function_data.e = all_functions[index*2+1][1]
        function_data.c = all_functions[index*2][2]
        function_data.f = all_functions[index*2+1][2]

        function_array.functions.append(function_data)


    functions = function_array.functions
    right_border_line = None
    left_border_line = None


    # detect border lines
    #################################
    for i in range(len(functions)):

        b, g, b = 255, 0, 0

        # get right and left border line
        if functions[i].position == 0:
            right_border_line = functions[i]
            b, g, b = 0, 255, 0
        elif functions[i].position == 1:
            left_border_line = functions[i]
            b, g, b = 0, 255, 0

        # draw functions to canvas
        x_coefficient = [None] * 3
        y_coefficient = [None] * 3
        x_coefficient[0] = functions[i].a
        x_coefficient[1] = functions[i].b
        x_coefficient[2] = functions[i].c
        y_coefficient[0] = functions[i].d
        y_coefficient[1] = functions[i].e
        y_coefficient[2] = functions[i].f
        
        if DISPLAY_CANVAS:
            for m in range(-250, 1500, 2):                             
                x = np.polyval(x_coefficient, m)
                y = np.polyval(y_coefficient, m)
                cv2.circle(canvas, (int(x), int(y)), 1, (b, g, b), -1)

    
    # calculate ideal line and offset
    #################################s[1, 0, k] = start_end_points[1, 0, i]
                    # start_end_point
    global TRACK_WIDTH

    if right_border_line is not None and left_border_line is not None:

        # left and right line are in range
        if 0.6 < right_border_line.x_position - left_border_line.x_position < TRACK_WIDTH * 1.4:
            ideal_x_coefficient = [None] * 3
            ideal_y_coefficient = [None] * 3
            ideal_x_coefficient[0] = (right_border_line.a + left_border_line.a) / 2
            ideal_x_coefficient[1] = (right_border_line.b + left_border_line.b) / 2
            ideal_x_coefficient[2] = (right_border_line.c + left_border_line.c) / 2
            ideal_y_coefficient[0] = (right_border_line.d + left_border_line.d) / 2
            ideal_y_coefficient[1] = (right_border_line.e + left_border_line.e) / 2
            ideal_y_coefficient[2] = (right_border_line.f + left_border_line.f) / 2
            for m in range(-250, 1500, 2):                             
                x = np.polyval(ideal_x_coefficient, m)
                y = np.polyval(ideal_y_coefficient, m)
                cv2.circle(canvas, (int(x), int(y)), 1, (0, 0, 255), -1)

            # determine offset
            #x(s) = as^2+bs+c
            a = ideal_x_coefficient[0]
            b = ideal_x_coefficient[1]
            c = ideal_x_coefficient[2]
            #y(s) = ds^2+es+f
            d = ideal_y_coefficient[0]
            e = ideal_y_coefficient[1]
            f = ideal_y_coefficient[2] - PICTURE_HEIGHT      # substract PICTURE_HEIGHT to find intersection with lower picture border, not the zeropoints

            # determine value of t at intersection with lower picture border -> quadratic formula (Mitternachtformel) 
            # quadratic formula (Mitternachtsformel)
            discriminant = math.sqrt(e**2 - (4 * d * f))
            
            t_1 = (-e - discriminant) / (2*d)
            # t_2 = (-e + discriminant) / (2*d)
            
            # determine x value for t value
            x_1 = np.polyval(ideal_x_coefficient, t_1)
            # x_2 = np.polyval(ideal_x_coefficient, t_2)
            
            offset_1 = x_1 - (TRUCK_POS_X)
            # offset_2 = x_2 - (TRUCK_POS_X)
            offset = offset_1

        # only right line is in range
        elif (TRACK_WIDTH/2)*0.8 < abs(right_border_line.x_position - TRUCK_POS_X) < (TRACK_WIDTH/2)*1.2:
            offset = right_border_line.x_position - (int(TRACK_WIDTH/2)) - TRUCK_POS_X
            cv2.line(canvas, (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT),
                    (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT - 20), (0, 0, 255), 5)
        # only left line is in range
        elif (TRACK_WIDTH/2)*0.8 < abs(left_border_line.x_position - TRUCK_POS_X) < (TRACK_WIDTH/2)*1.2:
            offset = right_border_line.x_position - (int(TRACK_WIDTH/2)) - TRUCK_POS_X
            cv2.line(canvas, (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT),
                (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT - 20), (0, 0, 255), 5)
        # neither left nor right lane is in range -> orient on right lane
        else:
            offset = right_border_line.x_position - (int(TRACK_WIDTH/2)) - TRUCK_POS_X
            cv2.line(canvas, (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT),
                (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT - 20), (0, 0, 255), 5)
    
    elif right_border_line is not None:
        offset = right_border_line.x_position - (int(TRACK_WIDTH/2)) - TRUCK_POS_X
        cv2.line(canvas, (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT),(int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT - 20), (0, 0, 255), 5)
    elif left_border_line is not None:
        offset = left_border_line.x_position - (int(TRACK_WIDTH/2)) - TRUCK_POS_X
        cv2.line(canvas, (int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT),(int(PICTURE_WIDTH/2) + offset, PICTURE_HEIGHT - 20), (0, 0, 255), 5)
    else:
        print("error: no border lines detected")
    time7 = time.time()
    if DISPLAY_TIME:
        print("time3 " + str(time7 - time6))


    # draw position of truck + offset value
    #################################
    if DISPLAY_CANVAS:
        cv2.line(canvas, (int(PICTURE_WIDTH/2), PICTURE_HEIGHT),
                    (int(PICTURE_WIDTH/2), PICTURE_HEIGHT - 20), (0, 0, 0), 5)
        cv2.putText(canvas, 'offset = ' + str(offset),( 20, PICTURE_HEIGHT - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0)
    
    
    # publish offset via ROS topic
    ################################
    time8 = time.time()
    talker(offset)
    time9 = time.time()
    if DISPLAY_TIME:
        print("time4 " + str(time9 - time8))

    
    # display canvas window
    #################################
    if DISPLAY_CANVAS:
        cv2.imshow("laneregression", canvas)
        # cv2.imwrite('result.png', canvas)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()
    time12 = time.time()
    if DISPLAY_TIME:
        print("time9 " + str(time12 - time1))


def listener():
    rospy.init_node('laneregression', anonymous=True)
    rospy.Subscriber("cluster_data", cluster_data, callback, queue_size=1)
    rospy.spin()


def talker(data_function):
    pub = rospy.Publisher('laneregression_offset', Float32, queue_size=1)
    #rate = rospy.Rate(30)  # hz
    if not rospy.is_shutdown():
        # rospy.loginfo(data_function)
        pub.publish(data_function)
        #rate.sleep()


if __name__ == '__main__':
    listener()