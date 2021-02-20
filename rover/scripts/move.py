#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Move:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.x_sub = rospy.Subscriber('x_error', Float64, self.x_callback)
        self.phi_sub = rospy.Subscriber('phi_error', Float64, self.phi_callback)
        self.twist = Twist()
        self.twist.linear.x = 0.00
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        self.x_error = -1
        self.phi_error = -1
        self.rate = rospy.Rate(10)
        self.z = 0
    
    def x_callback(self, msg):
        self.x_error = msg.data 
    
    def phi_callback(self, msg):
        self.phi_error = msg.data
        
    def P_feedback(self, verbose = False): #proportional-error feedback system
      #proportional gains
      kp_x = 4.5
      kp_phi = 1.5
      
      #fixed forward velocity
      self.twist.linear.x = 0.07
      
      if (self.x_error != -1):
        self.z= kp_x*float(self.x_error-320)/320.0 + kp_phi*float(self.phi_error)/90

      self.twist.angular.z = self.z
      self.pub.publish(self.twist)
      self.rate.sleep()
      
      if verbose:
        rospy.loginfo(str(self.twist.linear.x) + "|" + str(self.twist.angular.z))

        

def main(args=None):
    rospy.init_node('move', anonymous=True)
    move = Move()
    try:
        while True:
            move.P_feedback()
    except KeyboardInterrupt:
        move.twist.linear.x = 0
        move.twist.angular.z = 0
        move.pub.publish(move.twist)
        print("stopping")


if __name__ == '__main__':
    main()
