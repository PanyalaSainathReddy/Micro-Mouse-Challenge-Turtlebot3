#!/usr/bin/env python
import time
import numpy
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import array, inf


class TurtleBot:
    def __init__(self):
        # This constructor will initiate all the readings required navigation
        self.vel = Twist()

        # flag for knowing if the bot is out of maze
        self.out_of_maze = False
        self.turtlebot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.turtlebot_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ranges = []

        # initializing Regions with the inf readings 
        # here inf = 10 
        self.regions = {'bright': 10,
                        'right': 10,
                        'fright': 10,
                        'front': 10,
                        'fleft': 10,
                        'left': 10,
                        'bleft': 10, }
        self.surroundings = [self.regions['fleft'],
                             self.regions['fright'],
                             self.regions['left'],
                             self.regions['front'],
                             self.regions['right'],
                             self.regions['bright']]
        self.rate = rospy.Rate(5)  # 5hz

    def scan_callback(self, msg):

        # callback function after recieving the readings from /LasersScan topic
        self.ranges = array(msg.ranges)
        self.ranges[self.ranges == inf] = 10
        self.regions = {
            'bright': min(numpy.mean(self.ranges[234:270]), inf),
            'right': min(numpy.mean(self.ranges[270:280]), inf),
            'fright': min(numpy.mean(self.ranges[306:342]), inf),
            'front': min(numpy.mean(self.ranges[342:360] + self.ranges[0:18]), inf),
            'fleft': min(numpy.mean(self.ranges[18:54]), inf),
            'left': min(numpy.mean(self.ranges[54:90]), inf),
            'bleft': min(numpy.mean(self.ranges[90:126]), inf),
        }

    def algorithm(self):

        while not self.out_of_maze:
            
            # The minimum desired value from the wall is kept as 0.35
            # The values above 0.9 will become 2
            # The Values between 0.35 and 0.9 will become 1
            # The values less than 0.35 will become 0
            for key in self.regions.keys():
                if self.regions[key] >= 0.9:
                    self.regions[key] = 2
                elif 0.35 < self.regions[key] < 0.9:
                    self.regions[key] = 1
                else:
                    self.regions[key] = 0
            self.surroundings = [self.regions['fleft'],
                                 self.regions['fright'],
                                 self.regions['left'],
                                 self.regions['front'],
                                 self.regions['right'],
                                 self.regions['bright']]
            
            # moving right when the right region is inf and backright region is not inf
            if self.surroundings[4:] == [2, 1]:
                self.vel.angular.z = -1.5
                self.vel.linear.x = 0.2
            
            # moving right when the right region is inf and backright region is not inf but very near to wall
            elif self.surroundings[4:] == [2, 0]:
                self.vel.angular.z = -1.5
                self.vel.linear.x = 0.2
            
            # If there is wall in front
            elif self.surroundings[3] == 1:

                # rotating left if left values are greater than right values
                if self.surroundings[0] >= self.surroundings[1]:
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0.5
                
                # rotating right if right values are greater than left values
                else:
                    self.vel.angular.z = -0.5
                    self.vel.linear.x = 0.1
            
            # tilting somewhat right if it is too near to a wall on left side
            elif self.surroundings[0] == 0:
                self.vel.linear.x = 0.09
                self.vel.angular.z = -0.2
            
            # tilting somewhat left if it is too near to a wall on right side
            elif self.surroundings[1] == 0:
                self.vel.linear.x = 0.09
                self.vel.angular.z = 0.2
            
            # checking if it is out of maze
            elif self.surroundings[:4] == [2, 2, 2, 2, 2]:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
            
            # moving straight without any angular velocity if none of the condition satisfied
            else:
                self.vel.linear.x = 0.3
                self.vel.angular.z = 0
            self.turtlebot_pub.publish(self.vel)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('rosbot_test', anonymous=True)

    turtlebot = TurtleBot()

    try:
        turtlebot.algorithm()

    except rospy.ROSInterruptException:
        pass
