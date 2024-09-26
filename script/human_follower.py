#!/usr/bin/env python
#coding: utf-8

import rospy
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes

class MarkerFollower:

  def __init__(self):
    self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    self.sub = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback)
    
  def callback(self, msg):
    cmd_vel = Twist()
    for bb in msg.bounding_boxes:
      if bb.class == "person":
        x_position = (bb.x_min + bb.x_max)/2
        
        err_horizontal = 640/2 - x_position						#左右方向のずれ
        cmd_vel.angular.z = float(err_horizontal)*1 	#左右方向のずれに応じて旋回速度を変化させる
        self.pub.publish(cmd_vel)
        print("err_horizontal:", err_horizontal, "err_distance:", err_distance)
        return

if __name__ == "__main__":
  rospy.init_node("marker_follower")
  mf = MarkerFollower()
  rospy.spin()
