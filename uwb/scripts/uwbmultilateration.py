#!/usr/bin/env python

import rospy
from gtec_msgs.msg import Ranging
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from scipy.optimize import minimize
from scipy.optimize import least_squares
import numpy as np
import localization as lx
import random


from math import sqrt



class Multi_node:

    def __init__(self):

        self.ExponentialFilters = ExponentialFilter(0.2)
        self.MovingAverageFilterx = MovingAverageFilter(10)
        self.MovingAverageFiltery = MovingAverageFilter(10)
        self.MovingAverageFilterz = MovingAverageFilter(10)

        self.msg = Odometry()
        self.msgr = Float32()
        self.msgrf = Float32()

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/gtec/toa/ranging', Ranging, self.rangingCallback)
        self.pub = rospy.Publisher('odom_multi', Odometry, queue_size=1000)
        self.pub_nlos = rospy.Publisher('odom_multi_nlos', Odometry, queue_size=1000)
        self.pub_r = rospy.Publisher('/r', Float32, queue_size=100)
        self.pub_rf = rospy.Publisher('/rf', Float32, queue_size=100)
       
        #anchor 0,1,2,3
        self.msg_ranging = [0,0,0,0]
        self.anchor_pos = [[5,0,5],[-5,0,4],[0,5,3],[0,-5,2]]
        #gpt
        self.anchor_pos_a = np.array([[5,0,5],[-5,0,4],[0,5,3],[0,-5,2]])
        self.msg_ranging_a = np.array([0,0,0,0])

        # xiyizi -> k1k2k3k4
        self.k = [0,0,0,0,0]
        for i in range(4):
            for j in range(2):
                self.k[i] =+ self.anchor_pos[i][j]**2

        self.P=lx.Project(mode='3D',solver='LSE')
        self.P.add_anchor('anchore_A',(5,0,3))
        self.P.add_anchor('anchore_B',(-5,0,5))
        self.P.add_anchor('anchore_C',(0,5,5))
        self.P.add_anchor('anchore_D',(0,-5,1))
        self.t,self.label=self.P.add_target()

    def rangingCallback(self, msg):
            if msg.anchorId == 0 : self.msg_ranging[0] = msg.range/1000
            if msg.anchorId == 1 : self.msg_ranging[1] = msg.range/1000
            if msg.anchorId == 2 : self.msg_ranging[2] = msg.range/1000
            if msg.anchorId == 3 : self.msg_ranging[3] = msg.range/1000

            # if msg.anchorId == 0 : self.msg_ranging_a[0] = msg.range/1000
            # if msg.anchorId == 1 : self.msg_ranging_a[1] = msg.range/1000
            # if msg.anchorId == 2 : self.msg_ranging_a[2] = msg.range/1000
            # if msg.anchorId == 3 : self.msg_ranging_a[3] = msg.range/1000

            
    

    def main(self):

        rate = rospy.Rate(75)
        while not rospy.is_shutdown():
            # self.msg.pose.header.timestamp = rospy.get_rostime()
            

            self.t.add_measure('anchore_A',self.msg_ranging[0])
            self.t.add_measure('anchore_B',self.msg_ranging[1])
            self.t.add_measure('anchore_C',self.msg_ranging[2])
            self.t.add_measure('anchore_D',self.msg_ranging[3])


            if self.msg_ranging[0] != 0:
                
                # rospy.loginfo(self.trilaterate_3d(self.anchor_pos_a, self.msg_ranging_a))

                rospy.loginfo(self.gps_solve(list(self.anchor_pos_a), self.msg_ranging))
                self.msg.header.stamp = rospy.Time.now()
                self.msg.header.frame_id = "map"
                self.msg.child_frame_id = "odom"
                self.msg.pose.pose.position.z = self.MovingAverageFilterz.start(self.loc1()[2])
                self.msg.pose.pose.position.x = self.MovingAverageFilterx.start(self.loc1()[0])
                self.msg.pose.pose.position.y = self.MovingAverageFiltery.start(self.loc1()[1])


                # Filter testing only
                # self.msgr = self.msg.pose.pose.position.z
                # self.msgrf = self.MovingAverageFilterx.start(self.msgr)
                # self.pub_r.publish(self.msgr)
                # self.pub_rf.publish(self.msgrf) 

                self.pub.publish(self.msg)
                self.msg.pose.pose.position.z = self.loc1()[2] + (random.random()*0.2-0.1)
                self.msg.pose.pose.position.x = self.loc1()[0] + (random.random()*0.2-0.1)
                self.msg.pose.pose.position.y = self.loc1()[1] + (random.random()*0.2-0.1)
                self.pub_nlos.publish(self.msg)
            
            

            rate.sleep()

    def true_range_mult(self):

        pos = [0,0,0]

        u = self.anchor_pos[1][0] - self.anchor_pos[0][0]

        pos[0] = ((self.msg_ranging[0]**2 - self.msg_ranging[1]**2 + u**2)/2*u )
        pos[1] = (self.msg_ranging[0]**2 - self.msg_ranging[2]**2 + self.anchor_pos[2][0]**2 + self.anchor_pos[2][1]**2 - 2*self.anchor_pos[2][0]*pos[0])/2*self.anchor_pos[2][1]
        pos[2] = abs(sqrt(abs(self.msg_ranging[0]**2 - pos[0]**2 - pos[1]**2)))
        pos[2] -= 5

        return pos

    def gps_solve(self, distances_to_station, stations_coordinates):
        def error(x, c, r):
            return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])

        l = len(stations_coordinates)
        S = sum(distances_to_station)
        # compute weight vector for initial guess
        W = [((l - 1) * S) / (S - w) for w in distances_to_station]
        # get initial guess of point location
        x0 = sum([W[i] * stations_coordinates[i] for i in range(l)])
        # optimize distance from signal origin to border of spheres
        return minimize(error, x0, args=(stations_coordinates, distances_to_station), method='Nelder-Mead').x

    def loc1(self):


        A = np.array([[self.anchor_pos[1][0] - self.anchor_pos[0][0], self.anchor_pos[1][1] - self.anchor_pos[0][1], self.anchor_pos[1][2] - self.anchor_pos[0][2]], 
                      [self.anchor_pos[2][0] - self.anchor_pos[0][0], self.anchor_pos[2][1] - self.anchor_pos[0][1], self.anchor_pos[2][2] - self.anchor_pos[0][2]],
                      [self.anchor_pos[3][0] - self.anchor_pos[0][0], self.anchor_pos[3][1] - self.anchor_pos[0][1], self.anchor_pos[3][2] - self.anchor_pos[0][2]],
                      [self.anchor_pos[4][0] - self.anchor_pos[0][0], self.anchor_pos[4][1] - self.anchor_pos[0][1], self.anchor_pos[4][2] - self.anchor_pos[0][2]]])
        
        b = np.array([(self.msg_ranging[0]**2 - self.msg_ranging[1]**2 - self.k[0] + self.k[1]),
                      (self.msg_ranging[0]**2 - self.msg_ranging[2]**2 - self.k[0] + self.k[2]),
                      (self.msg_ranging[0]**2 - self.msg_ranging[3]**2 - self.k[0] + self.k[3]),
                      (self.msg_ranging[0]**2 - self.msg_ranging[4]**2 - self.k[0] + self.k[4])])
        r = np.dot(np.linalg.inv((np.dot(A.T,A))),np.dot(A.T,b))/2

        r[0] = r[0] - 0.43
        r[1] = r[1] - 1.7
        r[2] = r[2] + 5.8

        return r

    def trilaterate_3d(self,anchor_positions,distances):
        def residual(point, anchor_positions, distances):
            return [np.linalg.norm(point - anchor) - distance for anchor, distance in zip(anchor_positions, distances)]

        # Initial guess for the tag position (you might want to improve this based on your application)
        initial_guess = np.mean(anchor_positions, axis=0)

        # Use least squares optimization to find the tag position
        result = least_squares(residual, initial_guess, args=(anchor_positions, distances))

        tag_position = result.x
        return tag_position

class MovingAverageFilter:
    
    def __init__(self, MAF_level=4):
        self.MAF_level = MAF_level
        self.data = [0 for i in range(MAF_level)]
        print(self.data)

    def start(self, newData):
        self.data.pop()
        self.data.insert(0, newData)
        return (sum(self.data)/len(self.data))

class ExponentialFilter:
    
    def __init__(self, alpha=0.2):
        self.oldData = 0
        self.alpha = alpha

    def start(self, newData):
        result = self.alpha*newData + (1-self.alpha) * self.oldData
        self.oldData = result
        return result


    

if __name__ == '__main__':
    try:
        node = Multi_node()
        node.main()
    except rospy.ROSInterruptException:
        pass




