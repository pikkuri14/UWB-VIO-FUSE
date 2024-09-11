#!/usr/bin/env python3


import rospy
from nav_msgs.msg import Odometry
import random


class OdomSubscriber:
    def __init__(self):
        rospy.init_node('odom_subscriber', anonymous=True)
        rospy.Subscriber('camera/odom/sample', Odometry, self.odom_callback)
        self.pub_ground_truth = rospy.Publisher('ground_truth', Odometry, queue_size=1000)
        self.pub = rospy.Publisher('realsense_noisy', Odometry, queue_size=1000)
        self.realsense_noisy = Odometry()
        

    def odom_callback(self, msg):
        # Callback function to handle incoming odometry messages
        # rospy.loginfo(msg.pose.pose.position.x)

        self.realsense_noisy.pose.pose.position.x = msg.pose.pose.position.x -0.06
        self.realsense_noisy.pose.pose.position.y = msg.pose.pose.position.y 
        self.realsense_noisy.pose.pose.position.z = msg.pose.pose.position.z -0.59
        rospy.loginfo(f"x={self.realsense_noisy.pose.pose.position.x}, y={self.realsense_noisy.pose.pose.position.y}, z={self.realsense_noisy.pose.pose.position.z}, ")
        

        # Print the received information
        

    def run(self):
        # Spin to keep the script alive and receive messages

        rate = rospy.Rate(200)
        while not rospy.is_shutdown():


            self.realsense_noisy.header.stamp = rospy.Time.now()

            # self.realsense_noisy.pose.pose.position.x = self.realsense_noisy.pose.pose.position.x 
            # self.realsense_noisy.pose.pose.position.y = self.realsense_noisy.pose.pose.position.y 
            # self.realsense_noisy.pose.pose.position.z = self.realsense_noisy.pose.pose.position.z - 0.59
            self.pub_ground_truth.publish(self.realsense_noisy)
            rospy.loginfo(f"x={self.realsense_noisy.pose.pose.position.x}, y={self.realsense_noisy.pose.pose.position.y}, z={self.realsense_noisy.pose.pose.position.z}, ")

            self.realsense_noisy.header.frame_id = "map"
            self.realsense_noisy.child_frame_id = "base_footprint"
            self.realsense_noisy.pose.pose.position.x = self.realsense_noisy.pose.pose.position.x + random.random()*0.05-0.025
            self.realsense_noisy.pose.pose.position.y = self.realsense_noisy.pose.pose.position.y + random.random()*0.05-0.025
            self.realsense_noisy.pose.pose.position.z = self.realsense_noisy.pose.pose.position.z + random.random()*0.05-0.025 
            
            # rospy.loginfo(self.realsense_noisy)
            
            self.pub.publish(self.realsense_noisy)


            rate.sleep()

if __name__ == '__main__':
    try:
        odom_subscriber = OdomSubscriber()
        odom_subscriber.run()
    except rospy.ROSInterruptException:
        pass
