#!/usr/bin/env python
#coding: utf-8

import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class MarkerFollower:

  def __init__(self):
    self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    self.sub = rospy.Subscriber("camera/ar_pose_marker", AlvarMarkers, self.callback)
    
  def callback(self, msg):
    cmd_vel = Twist()
    for marker in msg.markers:
      if marker.id == 4:
        m_x = marker.pose.pose.position.x
        m_y = marker.pose.pose.position.y
        m_z = marker.pose.pose.position.z
        err_horizontal = m_y						#左右方向のずれ
        cmd_vel.angular.z = float(err_horizontal)*5	#左右方向のずれに応じて旋回速度を変化させる
        err_distance = m_x - 0.3					#30cm手前が目標地点
        if err_distance > 0:						#到着するまでは一定速度で前進
          cmd_vel.linear.x = 0.1
        self.pub.publish(cmd_vel)
        print("err_horizontal:", err_horizontal, "err_distance:", err_distance)
        return
    print("lost marker!")

if __name__ == "__main__":
  rospy.init_node("marker_follower")
  mf = MarkerFollower()
  rospy.spin()
