import numpy as np
import cv2
import random

picHeight = 960
picWidth = 1280
poly_degree = 2


def print_circles(points):
    r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
    for i in range (int(points.size / 2)):              # size ergibt sich aus x + y --> besser machen
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1,(r, g, b), -1)

file = open("all_cluster_without_perspective_transformation", "r")
for line in file:
    # white canvas
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    cluster = line.split(";")
    # go through clusters
    for n in range(len(cluster) - 1):
        fields = cluster[n].split(",")
        points = np.zeros((int((len(fields)- 1 )/2), 2))
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
                # find min und max x-value
                if int(fields[i]) < cluster_y_value_min:
                    cluster_y_value_min = int(fields[i])
                elif int(fields[i]) > cluster_y_value_max:
                    cluster_y_value_max = int(fields[i])
        # convert string to int
        points = points.astype(int)
        print_circles(points)

        # calculate function
        function = np.polyfit(points[:, 1], points[:, 0], poly_degree)

        # print all points of function line
        for i in range(int(cluster_y_value_min), int(cluster_y_value_max)):
            cv2.circle(canvas, (int(np.polyval(function, i)), i), 1, (0, 0, 0), -1)

        # print line from start to end point of function
        function_start_point = np.polyval(function, cluster_y_value_min)
        function_end_point = np.polyval(function, cluster_y_value_max)
        # cv2.line(canvas, (int(function_start_point), cluster_x_value_min) , (int(function_end_point), cluster_x_value_max),(0, 0, 0) , 1)

        # function string (polydegree, start point, end point)
        print(str(poly_degree) + "," + str(int(function_start_point)) + "," + str(cluster_y_value_min) + "," + str(int(function_end_point)) + "," + str(cluster_y_value_max))

    cv2.imshow("points", canvas)
    # cv2.imwrite("raw.png", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

file.close()