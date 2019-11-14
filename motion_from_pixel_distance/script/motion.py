#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import re

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class motion:

  def __init__(self):

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

    self.string_sub_steering = rospy.Subscriber("/pixel_distance/steering", String, self.callback_steering)
    self.string_sub_running = rospy.Subscriber("/pixel_distance/running", String, self.callback_running)

  def callback_steering(self, msg):
    # receive type String, "12,89"
    err_string = msg.data
    # split int from string, "12,89"
    err_ints = re.split(',', err_string);

    err_int_x = err_ints[0]

    self.twist.linear.x = 0
    self.twist.angular.z = -float(err_int_x) / 100
    self.cmd_vel_pub.publish(self.twist)
    print(self.twist)

    #
    # # BEGIN CONTROL
    # err = cx - 1*w/2
    # self.twist.linear.x = 1
    # dt = rospy.get_time() - ptime
    # self.twist.angular.z = (-float(err) / 100)*1 + ((err - perr)/(rospy.get_time() - 		ptime))*1/15/100 #+ (serr*dt)*1/20/100 #1 is best, starting 3 unstable
    # serr = err + serr
    # perr = err
    # ptime = rospy.get_time()
    #
    # self.cmd_vel_pub.publish(self.twist)

  def callback_running(self, msg):
       # receive type String, "12,89"
       err_string = msg.data
       # split int from string, "12,89"
       err_ints = re.split(',', err_string);

       err_int_x = err_ints[0]

       self.twist.linear.x = 0.2
       self.twist.angular.z = -float(err_int_x) / 100
       self.cmd_vel_pub.publish(self.twist)
       print(self.twist)


def main(args):
  rospy.init_node('motion_from_pixel_distance', anonymous=True)
  m = motion()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
