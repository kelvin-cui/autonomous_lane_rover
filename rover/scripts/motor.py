#!/usr/bin/env python
import rospy
import serial
import RPi.GPIO as GPIO

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Motor:

    def __init__(self):
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.motorcontrol)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.flush()
        
        #rover properties
        self.b = 0.157 #rover width between wheels
        self.r = 0.033 #wheel radius

    def motorcontrol(self, msg):
        rospy.loginfo("Linear Components: [%f, %f, %f]" %(msg.linear.x, msg.linear.y, msg.linear.x))
        rospy.loginfo("Angular Components: [%f, %f, %f]" %(msg.angular.x, msg.angular.y, msg.angular.z))
        rightvelocity = (msg.linear.x + 0.5*self.b*msg.angular.z)/self.r
        leftvelocity = (msg.linear.x - 0.5*self.b*msg.angular.z)/self.r
        self.ser.write(b"<" + str(rightvelocity) + "," + str(leftvelocity) + ">") #send command to arduino
        
        rospy.loginfo(str(rightvelocity) + str(leftvelocity))

def main(args=None):
    rospy.init_node('Motor', anonymous=True)
    motor = Motor()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("shutting down")
    
    

if __name__ == '__main__':
    main()
