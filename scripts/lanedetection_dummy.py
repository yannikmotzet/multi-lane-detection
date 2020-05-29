#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from lane_keeping_assist.msg import cluster_data
import os

def talker():
    pub = rospy.Publisher('cluster_data', cluster_data, queue_size=10)
    rospy.init_node('lanedetection_dummy', anonymous=True)
    rate = rospy.Rate(30) #hz

    while not rospy.is_shutdown():
        # print(os.getcwd())
        # file_cluster = open("./src/laneregression/scripts/all_cluster", "r")
        file_cluster = open("/home/zflab/catkin_ws/src/laneregression/scripts/all_cluster", "r")
        # file_cluster = open("./scripts/all_cluster_without_perspective_transformation", "r")      

        for line in file_cluster:
            # rospy.loginfo(topic_string)    

            all_cluster = cluster_data()
            
            string_cluster = line.split(";")            

            for i in range(len(string_cluster) - 1):

                string_points = string_cluster[i].split(",")

                all_cluster.size.append(int((len(string_points) - 1) / 2))
                
                for j in range(0, len(string_points) - 1, 2):
                    single_point = Point()
                    single_point.x = int(string_points[j])
                    single_point.y = int(string_points[j+1])
                    single_point.z = 0
                    all_cluster.points.append(single_point)


            # rospy.loginfo(all_cluster.size)
            pub.publish(all_cluster)
            rate.sleep()

        # topic_string = file_cluster.readline()

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass