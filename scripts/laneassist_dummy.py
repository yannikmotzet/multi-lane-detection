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

# Welches Bild ist hier gemeint? (Kamera, Video, Foto)
picHeight = 960                                                      # Bildhöhe
picWidth = 1280                                                      # Bildbreite
truck_pos_x = int(picWidth / 2)                                      # Truck wird in der Mitte des Bildes positioniert

# Diese beiden Zeilen verstehe ich leider nicht
def callback(data):
    rospy.loginfo("I recived " + str(int(len(data.functions))) + " polynomial(s).")

    functions = data.functions
    # Was bedeutet das None?
    right_border_line = None                                         # rechte Fahrbahnbegrenzungslinie
    left_border_line = None                                          # linke Fahrbahnbegrenzungslinie

    
    # draw functions to canvas
    #################################

    # Ist canvas das Koordinatensystem?
    canvas = 255 * np.ones(shape=[picHeight, picWidth, 3], dtype=np.uint8)

    for i in range(len(functions)):                                
        # Warum wird hier b überschrieben? Zuerst wird b = 127 zugewiesen und dann b = 0
        b, g, r = 127, 127, 0
        
        if functions[i].position == 0:                               # wenn die Position der Funktion = 0 ist, 
            right_border_line = functions[i]                         # ist die rechte Fahrbahnbegrenzungslinie = der Funktion
            b, g, r = 0, 255, 0                                      
        elif functions[i].position == 1:                             # wenn die Position der Funktion 0 1 ist,
            left_border_line = functions[i]                          # ist die linke Fahrbahnbegrenzungslinie = der Funktion 
            b, g, r = 0, 255, 0

        # Warum wird hier mit 3 multipliziert?
        x_coefficient = [None] * 3
        y_coefficient = [None] * 3
        # Was bedeuten die Zahlen 0,1,2 ? Waren das die Linienarten? 0 = durchgezogene Linie, 1 = gestrichelte Linie, 2 = ?
        # x-Werte 
        x_coefficient[0] = functions[i].a
        x_coefficient[1] = functions[i].b
        x_coefficient[2] = functions[i].c
        # y-Werte 
        y_coefficient[0] = functions[i].d
        y_coefficient[1] = functions[i].e
        y_coefficient[2] = functions[i].f

        for m in range(-250, 1500, 2):                  
            # Welche Polynom Funktion wird hier beschrieben?           
            x = np.polyval(x_coefficient, m)
            y = np.polyval(y_coefficient, m)
            cv2.circle(canvas, (int(x), int(y)), 1, (b, g, b), -1)

    
    # calculate ideal line
    #################################
    ideal_x_coefficient = [None] * 3
    ideal_y_coefficient = [None] * 3
    # x-Werte
    ideal_x_coefficient[0] = (right_border_line.a + left_border_line.a) / 2
    ideal_x_coefficient[1] = (right_border_line.b + left_border_line.b) / 2
    ideal_x_coefficient[2] = (right_border_line.c + left_border_line.c) / 2
    # y-Werte
    ideal_y_coefficient[0] = (right_border_line.d + left_border_line.d) / 2
    ideal_y_coefficient[1] = (right_border_line.e + left_border_line.e) / 2
    ideal_y_coefficient[2] = (right_border_line.f + left_border_line.f) / 2

    for m in range(-250, 1500, 2):                             
        x = np.polyval(ideal_x_coefficient, m)
        y = np.polyval(ideal_y_coefficient, m)
        # Müsste es in der Klammer nicht (0,225,0) heißen?, weil b,g,b in Zeile 43 und 46 so definiert wurde?
        cv2.circle(canvas, (int(x), int(y)), 1, (0, 0, 255), -1)

    # determine offset
    #################################
    
    #x(t)
    a = ideal_x_coefficient[0]
    b = ideal_x_coefficient[1]
    c = ideal_x_coefficient[2]
    #y(t)
    d = ideal_y_coefficient[0]
    e = ideal_y_coefficient[1]
    f = ideal_y_coefficient[2] - picHeight      # substract picHeight to find intersection with lower picture border, not the zeropoints

    # determine value of t at intersection with lower picture border -> quadratic formula (Mitternachtformel) 
    # quadratic formula (Mitternachtsformel)

    # Sind die Variablen in der Formel die idealen Koeffizienten?
    discriminant = math.sqrt(e**2 - (4 * d * f))
    
    t_1 = (-e + discriminant) / (2*d)
    t_2 = (-e - discriminant) / (2*d)
    
    # determine x value for t value
    x_1 = a*t_1**2 + b*t_1 + c
    x_2 = a*t_2**2 - b*t_2 + c
    
    # offset_1 = np.polyval(ideal_x_coefficient, x_1) - (picWidth/2)
    offset_2 = np.polyval(ideal_x_coefficient, x_2) - (picWidth/2)
    # print(offset_1)
    print("offset: " + str(offset_2))
    


    # draw position of truck
    #################################
    cv2.line(canvas, (int(picWidth/2), picHeight),
                 (int(picWidth/2), picHeight - 20), (0, 0, 0), 5)
    
    
    # display canvas window
    #################################
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