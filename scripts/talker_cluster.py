#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import os

def talker():
    pub = rospy.Publisher('chatter_cluster', String, queue_size=10)
    rospy.init_node('talker_cluster', anonymous=True)
    rate = rospy.Rate(0.5) #hz
    while not rospy.is_shutdown():
        
        print(os.getcwd())
        file_cluster = open("./scripts/all_cluster", "r")
        file_cluster = open("./scripts/all_cluster_without_perspective_transformation", "r")      

        for line in file_cluster:
            # rospy.loginfo(topic_string)
            pub.publish(line)
            rate.sleep()

        # topic_string = file_cluster.readline()

        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass