#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 as cv
import numpy as np
import itertools as it


from glob import glob
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




class person_recognition_hog:

  def __init__(self):

    self.image_pub = rospy.Publisher("/person_recognition_hog/output_video",Image, queue_size=1)
    self.string_pub = rospy.Publisher("/pixel_distance/running",String, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def inside(self, r, q):
      rx, ry, rw, rh = r
      qx, qy, qw, qh = q
      return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh


  def draw_detections(self, img, rects, thickness = 1):
      for x, y, w, h in rects:
          # the HOG detector returns slightly larger rectangles than the real objects.
          # so we slightly shrink the rectangles to get a nicer output.
          pad_w, pad_h = int(0.15*w), int(0.05*h)
          cv.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)


  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)



    hog = cv.HOGDescriptor()
    hog.setSVMDetector( cv.HOGDescriptor_getDefaultPeopleDetector() )

    found, w = hog.detectMultiScale(img, winStride=(8,8), padding=(32,32), scale=1.05)
    found_filtered = []
    for ri, r in enumerate(found):
        for qi, q in enumerate(found):
            if ri != qi and self.inside(r, q):
                break
        else:
            found_filtered.append(r)

    # calculate distance from image centroid
    size = img.shape
    pixel_distance = '0,0'
    for x, y, w, h in found_filtered:
        pixel_distance_x = "%s" % (x + w / 2 - size[0] / 2)
        pixel_distance_y = "%s" % (y + h / 2 - size[1] / 2)
        pixel_distance = pixel_distance_x + "," + pixel_distance_y


    self.draw_detections(img, found)
    self.draw_detections(img, found_filtered, 3)
    print('%d (%d) found' % (len(found_filtered), len(found)))
    cv.imshow('img', img)

    cv.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
      self.string_pub.publish(pixel_distance)
    except CvBridgeError as e:
      print(e)


def main(args):


  pfh = person_recognition_hog()

  rospy.init_node('person_recognition_hog', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
