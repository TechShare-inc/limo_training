#!/usr/bin/env python
#coding: utf-8

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleDetector:

	def __init__(self):
		
		self.obstacle_exist = False
		self.obstacle_threshold = 0.3
		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.vel_sub = rospy.Subscriber("cmd_vel_orig", Twist, self.vel_callback)	#元となる速度指令のトピックのSubscriber
		self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)		#LiDARのスキャンデータのSubscriber

	def scan_callback(self, msg):
		min_range = 100.0
		for i in range(len(msg.ranges)):        #ranges内の全ての要素を調べる
			if msg.ranges[i] != 0.0:            #0.0(測定不可)の点は無視する
				if msg.ranges[i] < min_range:       #i番目の点の距離が最小値より小さい場合 
					min_range = msg.ranges[i]       #最小値を更新する

		if min_range < self.obstacle_threshold:  #最小値が閾値より小さい場合
			self.obstacle_exist = True           #obstacle_existにTrueを代入
			print("Obstacle Exist")
		else:					            #最小値が閾値より大きい場合
			self.obstacle_exist = False          #obstacle_existにFalseを代入
			print("Obstacle don't Exist")

	def vel_callback(self, msg):
		vel = Twist()		#宣言した時点ではlinear, angularのx,y,zの全要素が0
		if not self.obstacle_exist:
			vel = msg		#障害物が存在しない場合は、cmd_vel_origの速度指令で上書き
		self.vel_pub.publish(vel)

if __name__ == "__main__":
	rospy.init_node("obstacle_detector")
	od = ObstacleDetector()
	rospy.spin()
