#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from laneregression.msg import point, clusterData
import os

def talker():
    pub = rospy.Publisher('lanedetection_cluster', clusterData, queue_size=10)
    rospy.init_node('lanedetection_dummy', anonymous=True)
    rate = rospy.Rate(0.5) #hz

    while not rospy.is_shutdown():
        # print(os.getcwd())
        file_cluster = open("./src/laneregression/scripts/all_cluster", "r")
        # file_cluster = open("/home/zflab/catkin_ws/src/laneregression/scripts/all_cluster", "r")
        # file_cluster = open("./scripts/all_cluster_without_perspective_transformation", "r")      

        for line in file_cluster:
            # rospy.loginfo(topic_string)    

            all_cluster = clusterData()
            
            string_cluster = line.split(";")            

            for i in range(len(string_cluster) - 1):

                string_points = string_cluster[i].split(",")

                all_cluster.size.append(int((len(string_points) - 1) / 2))
                
                for j in range(0, len(string_points) - 1, 2):
                    single_point = point()
                    single_point.x = int(string_points[j])
                    single_point.y = int(string_points[j+1])
                    all_cluster.points.append(single_point)


            rospy.loginfo(all_cluster)
            pub.publish(all_cluster)
            rate.sleep()

        # topic_string = file_cluster.readline()

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass