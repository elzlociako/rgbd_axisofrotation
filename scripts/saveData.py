#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import pandas as pd 
import numpy as np

# pickle albo numpy.save do ustawienia rozszerzenia
np.set_printoptions(threshold=sys.maxsize)
pd.set_option('display.max_rows', 1000)
pd.set_option('display.max_columns', 1000)
pd.set_option('display.width', 1000)
pd.set_option('display.max_colwidth', 1000)

first_taken = False
counter = 0

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub1 = rospy.Subscriber("/camera/rgb/image_raw",Image, self.callback_rgb)
    self.image_sub2 = rospy.Subscriber("/camera/depth_registered/image", Image, self.callback_depth)

  def callback_rgb(self,data):
    global cv_image_d, cv_image_rgb
    try:
      cv_image_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
      # print(data.data)

    except CvBridgeError as e:
      print(e)
    key = cv2.waitKey(1)
    cv2.imshow("Image window RGB", cv_image_rgb)
    if key == ord('s'):
      saving_images()
        
  def callback_depth(self,data):
    global cv_image_d, cv_image_rgb
    try:
      cv_image_d = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e)

def saving_images():
  global first_taken, counter
  counter_2 = counter - 1
  if(first_taken == False):
    cv2.imwrite('files/images/rgb_img1/BGR%05d.png'%counter, cv_image_rgb)
    np.save('files/images/depth_img1/D%05d'%counter, cv_image_d)
    first_taken = True
    print("First image was taken")
  else:
    cv2.imwrite('files/images/rgb_img2/BGR%05d.png'%counter, cv_image_rgb)
    np.save('files/images/depth_img2/D%05d'%counter, cv_image_d)
    first_taken = False
    print("Second image was taken")
    collect_data()
    counter += 1

def collect_data():
  global counter
  df = pd.DataFrame([['BGR%05d.png'%counter,
  'D%05d.npy'%counter,'BGR%05d.png'%counter, 'D%05d.npy'%counter]],
   columns=["rgb_img1", "depth_img1","rgb_img2", "depth_img2"])

  if not os.path.isfile('files/data.csv'):
    df.to_csv('files/data.csv', index=False)
  else: # else it exists so append without writing the header
    df.to_csv('files/data.csv', mode='a', header=False, index=False)

def set_counter():
  global counter
  csv_file = pd.read_csv('files/data.csv', usecols=["rgb_img1", "depth_img1","rgb_img2", "depth_img2"])
  last_element = csv_file["rgb_img1"].iloc[-1]
  num = ""
  for c in last_element:
      if c.isdigit():
          num = num + c
  counter = int(num) + 1

def main(args):
  if os.path.isfile('files/data.csv'):
    set_counter()

  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)