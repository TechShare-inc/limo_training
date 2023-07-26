#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("vel_pub")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    for i in range(10):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.0
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
