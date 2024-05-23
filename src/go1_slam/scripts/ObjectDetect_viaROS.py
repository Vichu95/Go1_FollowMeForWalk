#!/home/unitree/Documents/python38_venv/bin/python
#/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from ultralytics import YOLO



class image_converter:

  def __init__(self):

    self.image_sub = rospy.Subscriber("/camera/front/image",Image,self.callback)

    self.model = YOLO('yolov8n.pt')

    # Initialize variables to calculate average FPS and range
    self.total_fps = 0
    self.max_fps = float('-inf')
    self.min_fps = float('inf')
    self.num_results = 0

  def callback(self,data):	
    width = data.width
    height = data.height
    img_data = np.frombuffer(data.data, dtype=np.uint8)
    image = img_data.reshape((height, width, 3))
    # image_cv = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert RGB to BGR if needed


    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    # cv2.circle(image, (50,50), 10, 255)
	
    #cv2.imshow("Image window", image)
    #cv2.waitKey(1)
    results = self.model.track(image, show=True)
    # View results
    for r in results:
        #print(r.speed)  
        #print(r.speed['preprocess'])  
        total_speed = sum(r.speed.values())
        fps = 1000/total_speed
        print("FPS : ",fps) 

        self.num_results += 1  # Increment number of results

        # Calculate average FPS
        if self.num_results > 1:            
            self.total_fps += fps  # Update total FPS
            self.max_fps = max(self.max_fps, fps)  # Update maximum FPS
            self.min_fps = min(self.min_fps, fps)  # Update minimum FPS
            average_fps = self.total_fps / (self.num_results - 1)
            print("Average FPS = ", average_fps ," [ ", self.min_fps," , ", self.max_fps," ]")

    if(self.num_results > 25):
      print(self.num_results)
      # When you want to exit the node, call rospy.signal_shutdown()
      rospy.signal_shutdown("Your shutdown message")

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
 
    
    
  
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
