# ROS client library for Python
import rclpy

# Used to create nodes
from rclpy.node import Node

# Enables the use of the float message type
from sensor_msgs.msg import Image

# Allows conversion between ROS and OpenCV Images
from cv_bridge import CvBridge

# OpenCV library
import cv2
import numpy as np

class Camera(Node):
  """
  This is the Drive class we will use. 
  The class is a subclass of the Node class for ROS2.
  This class will control the robot motion
  """
  
  def __init__(self):
    """
    This is the class's constructor, used to initialize the node
    """
    # Initiate the parent Node class's constructor and give it a name
    super().__init__('camera')
    
    ########### SETUP NODE'S SUBSCRIBERS & PUBLISHERS ############
                               
    # Create a publisher to publish the desired distance
    # that's read from the mounted ultrasonic sensor (on the
    # robot chassis) to the /distance topic with value /distance/data. 
    self.publisher_ = self.create_publisher(
                      Image, 
                      '/camera_image', 
                      10)
  
    ################### CAMERA CONTROL PARAMETERS ##################
    
    # Create a timer that calls the get_image() function 30 
    # times per second. This allows for proper topic publishing
    self.create_timer(0.0333333, self.get_image)
    
    # Create a VideoCapture object to receive feed from
    # COM port 0 on the device
    self.img = cv2.VideoCapture(0)
    
    # Create the bridge between ROS and OpenCV
    self.br = CvBridge()
                              
  ###########  SUBSCRIPTION CALLBACK & HELPER FUNCTIONS ######## 
    
  def get_image(self):
  
    # Grab a frame from the video capture and store
    # it into 'frame'
    ret, frame = self.img.read()
    
    # If a frame has been captured, then publish it
    if(ret):
    	self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
    
    #print("Publishing video frame...")
  
########### MAIN FUNCTION ###############
# The main function is called automatically when the 
# ping node is launched. It is the entry point 
# for the node

def main(args=None):
    
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node (initializes an instance of the camera class)
    camera = Camera()
    
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(camera)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
