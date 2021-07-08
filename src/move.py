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
        self.vel = Twist()
        self.out_of_maze = False
        self.turtlebot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.turtlebot_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ranges = []
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
        self.rate = rospy.Rate(5)  # 1hz

    def scan_callback(self, msg):
        # self.ranges = list(msg.ranges)
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
            # print "left:", self.left
            # # print "front_avg:", self.front_average
            # # print "front", self.front
            print "regions:", self.regions
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
            print "surroundings:", self.surroundings
            if self.surroundings[4:] == [2, 1]:
                self.vel.angular.z = -1.5
                self.vel.linear.x = 0.2
            elif self.surroundings[4:] == [2, 0]:
                self.vel.angular.z = -1.5
                self.vel.linear.x = 0.2
            elif self.surroundings[3] == 1:
                if self.surroundings[0] >= self.surroundings[1]:
                    self.vel.linear.x = 0.1
                    self.vel.angular.z = 0.5
                else:
                    self.vel.angular.z = -0.5
                    self.vel.linear.x = 0.1
            elif self.surroundings[0] == 0:
                self.vel.linear.x = 0.09
                self.vel.angular.z = -0.2
            elif self.surroundings[1] == 0:
                self.vel.linear.x = 0.09
                self.vel.angular.z = 0.2
            elif self.surroundings[:4] == [2, 2, 2, 2, 2]:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
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
