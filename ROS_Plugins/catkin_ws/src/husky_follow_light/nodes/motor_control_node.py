#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class MotorControlNode:

    def __init__(self):
        rospy.init_node('motor_control_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=10)
        self.action_sub = rospy.Subscriber('/perform_action', Float32, self.action_callback)
        rospy.loginfo("Motor Control Node Initialized")

    def action_callback(self, msg):
        decision = msg.data
        rospy.loginfo(f"Received angle: {decision}" )
        self.move(decision)

    def move(self, decision):
        twist = Twist()
        if decision >= 0.5 or decision <= -0.5:
            twist.linear.x = 0.05
        else:
            twist.linear.x = 0.7
        twist.angular.z = decision
        
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    try:
        mcn = MotorControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
