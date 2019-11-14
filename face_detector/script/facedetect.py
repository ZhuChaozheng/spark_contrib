#!/usr/bin/env python

'''
face detection using haar cascades

USAGE:
    facedetect.py [--cascade <cascade_fn>] [--nested-cascade <cascade_fn>] [<video_source>]
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

import sys
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# local modules
from video import create_capture
from common import clock, draw_str

class facedetect:
    def __init__(self):
      self.image_pub = rospy.Publisher("/facedetect/output_video", Image, queue_size=1)
      self.string_pub = rospy.Publisher("/speak_string", String, queue_size=1)

      self.bridge = CvBridge()
      self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
      self.haar_face_path = rospy.get_param("/face_detector/haar_face_path")


    def detect(self, img, cascade):
        rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
        if len(rects) == 0:
            return []
        rects[:,2:] += rects[:,:2]
        return rects

    def draw_rects(self, img, rects, color):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

    def callback(self,data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # cascade = cv2.CascadeClassifier("../data/haarcascades/haarcascade_frontalface_alt.xml")
        cascade = cv2.CascadeClassifier(self.haar_face_path)
        # nested = cv2.CascadeClassifier("../data/haarcascades/haarcascade_eye.xml")

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        t = clock()
        rects = self.detect(gray, cascade)
        length_rects = str(len(rects))
        vis = image.copy()
        self.draw_rects(vis, rects, (0, 255, 0))
        # if not nested.empty():
        #     for x1, y1, x2, y2 in rects:
        #         roi = gray[y1:y2, x1:x2]
        #         vis_roi = vis[y1:y2, x1:x2]
        #         subrects = self.detect(roi.copy(), nested)
        #         self.draw_rects(vis_roi, subrects, (255, 0, 0))
        dt = clock() - t

        draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
        cv2.imshow('facedetect', vis)
        cv2.waitKey(1)

        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
          self.string_pub.publish(length_rects)
        except CvBridgeError as e:
          print(e)

def main():
  fd = facedetect()
  rospy.init_node('facedetector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
