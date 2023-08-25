#!/usr/bin/env python
#coding: utf-8

import rospy, cv_bridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np

class ColorFollower:

	def __init__(self):

		self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
		self.sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)
		self.bridge = cv_bridge.CvBridge()
		cv.namedWindow('BGR Image', 1)	#'BGR Image'という名前の画像表示のウィンドウを作成
		cv.namedWindow('MASK', 1)	#'MASK'という名前の画像表示のウィンドウを作成
		
	def callback(self, msg):

		cmd_vel = Twist()
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		h, w = image.shape[:2]
		RESIZE = (w//2, h//2)
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
		lower_hsv = np.array([95, 10, 10])		#フィルタの閾値（下限）
		upper_hsv = np.array([130, 255, 250])	#フィルタの閾値（上限）
		mask = cv.inRange(hsv, lower_hsv, upper_hsv)	#閾値による二値化画像の生成（フィルタ範囲内のピクセルは1として扱われる）
		area_top = h - 10
		area_bot = h				#検出範囲を画像の下から10pxに絞る
		mask[0:area_top, 0:w] = 0	
		mask[area_bot:h, 0:w] = 0	#検出範囲外のピクセルを0で上書きする
		
		gp = cv.moments(mask)	#maskにおけるライン領域(1のピクセル)の重心
		if gp['m00'] > 0:		#重心が存在する場合
		
			gp_x = int(gp['m10']/gp['m00'])	#重心のx座標
			gp_y = int(gp['m01']/gp['m00'])	#重心のy座標
			cv.circle(image, (gp_x, gp_y), 10, (0, 0, 255), -1)	#重心をBGR画像に表示
			err = gp_x - w//2		#ライン領域の重心の中心からのずれ
			cmd_vel.linear.x = 0.1	
			cmd_vel.angular.z = -float(err)/500	#ずれに応じて旋回速度を変化させる
			self.pub.publish(cmd_vel)

		display_image = cv.resize(image, RESIZE)
		display_mask = cv.resize(mask, RESIZE)
		cv.imshow('BGR Image', display_image)	#'BGR Image'ウィンドウにimageを表示
		cv.imshow('MASK', display_mask)			#'MASK'ウィンドウにimageを表示
		cv.waitKey(3)



if __name__ == "__main__":
	rospy.init_node("color_follower")
	cf = ColorFollower()
	rospy.spin()
