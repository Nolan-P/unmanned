#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import csv


class Logger:
	def __init__(self):
		self.sub1 = rospy.Subscriber('/turtlebot1/odom', Odometry, self.callback1)
		self.sub2 = rospy.Subscriber('/turtlebot2/odom', Odometry, self.callback2)
		self.path1 = '/home/nolan/Documents/unmanned/logging/homework6_tbot1.csv'
		self.path2 = '/home/nolan/Documents/unmanned/logging/homework6_tbot2.csv'
		self.data1 = []
		self.data2 = []


	def Log(self):
		frequency = 5
		rate = rospy.Rate(frequency)
		while not rospy.is_shutdown():
			with open(self.path1, 'a') as file:
				writer = csv.writer(file)
				writer.writerow(self.data1)

			with open(self.path2, 'a') as file:
				writer = csv.writer(file)
				writer.writerow(self.data2)
			rate.sleep()


	def callback1(self, msg):
		self.data1 = [msg.pose.pose.position.x, msg.pose.pose.position.y]

	def callback2(self, msg):
		self.data2 = [msg.pose.pose.position.x, msg.pose.pose.position.y]


if __name__ == '__main__':
	rospy.init_node('logger', anonymous=True)
	logger = Logger()
	logger.Log()
