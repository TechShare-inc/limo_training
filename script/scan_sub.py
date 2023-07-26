#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #LaserScanメッセージをインポート

obstacle_exist = False      #障害物の有無を格納する変数
obstacle_threshold = 0.5    #障害物検知の閾値（単位：m）

def callback(msg):
    min_range = 100.0
    global obstacle_exist
    for i in range(len(msg.ranges)):        #ranges内の全ての要素を調べる
        if msg.ranges[i] != 0.0:            #0.0(測定不可)の点は無視する
            if msg.ranges[i] < min_range:       #i番目の点の距離が最小値より小さい場合 
                min_range = msg.ranges[i]       #最小値を更新する

    if min_range < obstacle_threshold:  #最小値が閾値より小さい場合
        obstacle_exist = True           #obstacle_existにTrueを代入
        print("Obstacle Exist")
    else:					            #最小値が閾値より大きい場合
        obstacle_exist = False          #obstacle_existにFalseを代入
        print("Obstacle don't Exist")

def main():
    rospy.init_node("vel_pub")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber(“scan”, LaserScan, callback)  #Subscriberを宣言
    rate = rospy.Rate(10)
    for i in range(100):
        msg = Twist()
        if obstacle_exist:		    #obstacle_existがTrueの場合
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:				        #obstacle_existがFalseの場合
            msg.linear.x = 0.3
            msg.angular.z = 0.0
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
