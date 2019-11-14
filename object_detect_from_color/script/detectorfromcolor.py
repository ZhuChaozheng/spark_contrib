#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np


from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class detector_from_color:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detector_from_color/output_video",Image, queue_size=1)
    self.string_pub = rospy.Publisher("/pixel_distance/running",String, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    size = image.shape
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # HSV (hue, saturation, value) colorspace is a model to represent
    # the colorspace similar to the RGB color model. Since the hue
    # channel models the color type, it is very useful in image
    # processing tasks that need to segment objects based on its
    # color. Variation of the saturation goes from unsaturated to
    # represent shades of gray and fully saturated (no white component).
    # Value channel describes the brightness or the intensity of the color.

    ## cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    # if you want to track the red toy, please uncomment the next line
    # mask = cv.inRange(hsv, (0, 43, 46), (10, 255, 255))

    # the range is for skin color
    mask = cv.inRange(hsv, (7, 28, 50), (20, 256, 256))
    # Rectangular Kernel
    line = cv.getStructuringElement(cv.MORPH_RECT, (15, 15), (-1, -1))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, line)

    # fetch contours, find biggest contours
    out, contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    index = -1
    max = 0
    for c in range(len(contours)):
        area = cv.contourArea(contours[c])
        if area > max:
            max = area
            index = c
    # draw
    if index >= 0:
        rect = cv.minAreaRect(contours[index])
        cv.ellipse(image, rect, (0, 255, 0), 2, 8)
        cv.circle(image, (np.int32(rect[0][0]), np.int32(rect[0][1])), 2, (255, 0, 0), 2, 8, 0)

        # calculate distance from image centroid
        pixel_distance_x = "%s" % (np.int32(rect[0][1]) - (size[0]/2))
        pixel_distance_y = "%s" % (np.int32(rect[0][0]) - (size[1]/2))
        pixel_distance = pixel_distance_x + "," + pixel_distance_y

    cv.imshow("output", image)
    cv.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
      self.string_pub.publish(pixel_distance)
    except CvBridgeError as e:
      print(e)


def main(args):
  dfc = detector_from_color()
  rospy.init_node('detector_from_color', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
