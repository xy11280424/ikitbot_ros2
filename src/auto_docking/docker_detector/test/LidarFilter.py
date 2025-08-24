#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class DoFilter:
    def __init__(self):

        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("filteredscan", LaserScan, queue_size=10)

    def callback(self, data):

        newdata = data
        newdata.ranges = list(data.ranges)
        newdata.intensities = list(data.intensities)

        #通过清除不需要的扇区的数据来保留有效的数据
        # for x in range(120,240):
        #     newdata.ranges[x]=0
        #     newdata.intensities[x]=0

        #前方180°的扇区
        #for x in range(90,270):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0

        #正前方60°的扇区
        # for x in range(30,330):
        #    newdata.ranges[x]=0
        #    newdata.intensities[x]=0
        for i in range(len(newdata.intensities)):
            if (newdata.intensities[i] < 130) :
                newdata.ranges[i] = 0
                newdata.intensities[i] = 0

        self.pub.publish(newdata)
        rospy.loginfo(data.intensities)


if __name__ == '__main__':

    # Initialize
    rospy.init_node('LidarFilter', anonymous=False)
    lidar = DoFilter()

    rospy.spin()
