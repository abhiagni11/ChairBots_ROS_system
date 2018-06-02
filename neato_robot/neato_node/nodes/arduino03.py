#!/usr/bin/python

import roslib
import rospy
import serial
import syslog
import time
import datetime
import calendar
import sys, os
from std_msgs.msg import Int8

from geometry_msgs.msg import Twist
import time
from std_msgs.msg import UInt16
global send
send = False

def sending(var):
    global send
    if (var.data == 1):
        send = True
    elif (var.data == 0):
        send = False

def run():
    port = rospy.get_param('~port1', "/dev/ttyACM0")
    ard = serial.Serial(port, 9600, timeout=5)
    global send
    print 'Message from arduino: '
    while 1:
        try:
            msg = ard.readline()
            splits = msg.split(' ') 
            #print splits
            if (splits[1] == '1'):
            	touch = int(splits[0])
            	pub_touch.publish(touch)
            	print(touch)
            elif (splits[1] == '2'):
            	release = int(splits[0]) + 12
            	pub_touch.publish(release)
            	print(release)

            if (send):
                ard.write('i')
                #print('sending')   
            else:
                ard.write('o')
                #print('stopping')
            #pub_touch.publish(int(msg))
        except IndexError:
            continue
        except KeyboardInterrupt:
            sys.exit()

if __name__ == '__main__':

    rospy.init_node('arduino03', anonymous=True)
    pub_touch = rospy.Publisher('/touches03', Int8, queue_size=10)
    rospy.Subscriber("/led03", Int8, sending, queue_size=10)
    try:
        run()
    except rospy.ROSInitException:
        pass


			
