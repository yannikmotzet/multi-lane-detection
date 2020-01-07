import numpy as np
import cv2
import random

picHeight = 960
picWidth = 1280


def print_circles(points):
    r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
    for i in range (int(points.size / 2)):              # size ergibt sich aus x + y --> besser machen
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1,(r, g, b), -1)
    # draw line from startpoint to end point    
    # points_number = int(points.size / 2) - 1 - 50
    # start_point = points[20, 0], points[20, 1]
    # end_point = points[points_number, 0] , points[points_number, 1]
    # cv2.line(canvas, start_point, end_point, (255, g, b), 2)

def print_circles_blue(points):
    for i in range (int(points.size / 2)):
        r, g, b = 0, 0, 255
        cv2.circle(canvas, (points[i, 0], points[i, 1]), 1,(r, g, b), -1)


# clusters
file_left_border = open("all_cluster", "r")
for line in file_left_border:
    # white canvas
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    cluster = line.split(";")
    # print(cluster[0])
    for n in range(len(cluster) - 1):
        fields = cluster[n].split(",")
        points = np.zeros((int((len(fields)- 1 )/2), 2))
        for i in range(len(fields) - 1):              # last field is empty
            # x-value
            if (i % 2 == 0):
                points[(int(i/2)), 0] = fields[i]
            # y-value
            else:
                points[(int((i-1)/2)), 1] = fields[i]
        points = points.astype(int)
        print_circles(points)
    cv2.imshow("points", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()



    # if k <= 0:
    #     fields = line.split(";")
    #     points = np.zeros((int((len(fields)-1)/2), 2))
    #     for i in range(len(fields)-1):              # last field is empty
    #         if (i % 2 == 0):
    #             points[(int((i-1)/2)), 0] = fields[i]
    #         else:
    #             points[(int(i/2)), 1] = fields[i]
    #     points = points.astype(int)
    #     print_circles(points)
    #     k = 2
    #     cv2.imshow("points", canvas)
    #     cv2.waitKey(1000)
    #     cv2.destroyAllWindows()
    # elif k <= 2:
    #     canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)
    #     fields = line.split(";")
    #     points = np.zeros((int((len(fields)-1)/2), 2))
    #     for i in range(len(fields)-1):              # last field is empty
    #         if (i % 2 == 0):
    #             points[(int((i-1)/2)), 0] = fields[i]
    #         else:
    #             points[(int(i/2)), 1] = fields[i]
    #     points = points.astype(int)
    #     print_circles(points)
    #     k = k-1


file_left_border.close()

# # clusters of right border
# file_right_border = open("right_border", "r")
# for line in file_right_border:
#    fields = line.split(";")
#    points = np.zeros((int((len(fields)-1)/2), 2))
#    for i in range(len(fields)-1):              # last field is empty
#        if (i % 2 == 0):
#            points[(int((i-1)/2)), 0] = fields[i]
#        else:
#            points[(int(i/2)), 1] = fields[i]
#    points = points.astype(int)
#    print_circles_blue(points)
# file_right_border.close()



# cv2.imshow("visualization of clustering", canvas)
# cv2.waitKey(0)
# cv2.destroyAllWindows()