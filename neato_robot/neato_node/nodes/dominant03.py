#!/usr/bin/env python

import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos

from geometry_msgs.msg import Twist
import time
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy

from neato_driver.neato_driver import Botvac

class NeatoNode:

    def __init__(self):
    	""" Start up connection to the Neato Robot. """
    	rospy.init_node('teleop03', anonymous=True)

    	self.port = rospy.get_param('~port', "/dev/ttyACM0")
    	rospy.loginfo("Using port: %s"%(self.port))

    	self.robot = Botvac(self.port)

    	rospy.Subscriber("/joy03", Joy , self.joy_handler, queue_size=10)

        self.Axess = (-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0)
        self.Butt = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.SPEED = 0
        self.DIST = 20
	'''
	def optiona(self):
	  #forward
	  self.SPEED = 300
	  #right
	  self.robot.setMotors(-50,50,self.SPEED)
	  rospy.sleep(1)
	  #left
	  self.robot.setMotors(100,-100,self.SPEED)
	  rospy.sleep(1)
	  self.robot.setMotors(-50,50,self.SPEED)
	  rospy.sleep(1)
	def optionb(self):
	  self.SPEED=100
	  self.robot.setMotors(-100,-100,self.SPEED)
	  rospy.sleep(1)
	  self.robot.setMotors(100,100,self.SPEED)
	  rospy.sleep(1)
	def optionc(self):
	  self.SPEED=200
	  self.robot.setMotors(-100,-100,self.SPEED/2)
	  rospy.sleep(1)
	  self.robot.setMotors(200,200,self.SPEED*(1.5))
	  rospy.sleep(1)
	def optiond(self):
	  self.SPEED=200
	  for _ in range(2):
	    self.robot.setMotors(-100,-100,self.SPEED)
	    rospy.sleep(1)
	    self.robot.setMotors(100,100,self.SPEED)
	    rospy.sleep(1)
	'''
    def spin(self):
        
        # main loop of driver
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            Lft_t = self.Axess[0]
            Lft_d = self.Axess[1]
            Rgh_t = self.Axess[3]
            Rgh_d = self.Axess[4]
            AageL = self.Axess[2]
            AageR = self.Axess[5]
            L_R = self.Axess[6]
            F_B = self.Axess[7]
            sq = self.Butt[0]
            xx = self.Butt[1]
            ci = self.Butt[2]
            tr = self.Butt[3]
            self.SPEED_s = self.Butt[4]
            self.SPEED_f = self.Butt[5]
            AageL_Button = self.Butt[6]
            AageR_Button = self.Butt[7]
            share = self.Butt[8]
            options = self.Butt[9]
            pressL = self.Butt[10]
            pressR = self.Butt[11]
            power = self.Butt[12]
            self.SPEED -= ((AageR-1)*10)
            self.SPEED += ((AageL-1)*10)
            self.SPEED = int(self.SPEED)
            if (self.SPEED<0):
                self.SPEED=0
            elif (self.SPEED>330):
                self.SPEED=330
            ll = (Lft_d*self.DIST)
            rr = (Rgh_t*self.DIST)
            if (rr>=0):
                x = (-ll - rr)
                y = (-ll + rr)
            else:
                x = (-ll - rr)
                y = (-ll + rr) 

            x=int(x)
            y=int(y)
  
            print (x,y,self.SPEED)
            self.robot.setMotors(x,y,self.SPEED)
            self.robot.flushing()
            '''
            if tr == 1:
    			self.optiona()
			if ci == 1:
				self.optionb()
			if xx == 1:
				self.optionc()
			if sq == 1:
				self.optiond()
            '''
            # wait, then do it again
            r.sleep()

        # shut down
        self.robot.setLDS("off")
        self.robot.setTestMode("off") 

    def joy_handler(self, ps):
        self.Butt =  ps.buttons
        self.Axess = ps.axes

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()

