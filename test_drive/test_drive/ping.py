# ROS client library for Python
import rclpy

# Used to create nodes
from rclpy.node import Node

# Enables pauses in the execution of code
from time import sleep 

# Enables the use of the float message type
from std_msgs.msg import Float64

import math
import numpy as np
import RPi.GPIO as GPIO
import time

class Ping(Node):
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
    super().__init__('ping')
    
    ########### SETUP NODE'S SUBSCRIBERS & PUBLISHERS ############
                               
    # Create a publisher to publish the desired distance
    # that's read from the mounted ultrasonic sensor (on the
    # robot chassis) to the /distance topic with value /distance/data. 
    self.publisher_ = self.create_publisher(
                      Float64, 
                      '/distance', 
                      10)
  
  ################### ROBOT CONTROL PARAMETERS ##################
    
    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)
 
    #set GPIO Pins
    self.GPIO_TRIGGER = 18
    self.GPIO_ECHO = 24
 
    #set GPIO direction (IN / OUT)
    GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(self.GPIO_ECHO, GPIO.IN)
    
    #Create a timer that calls the get_distance() function 3 
    #times per second. This allows for proper topic publishing
    self.create_timer(0.33333, self.get_distance)
                              
  ###########  SUBSCRIPTION CALLBACK & HELPER FUNCTIONS ######## 
    
  def get_distance(self):
  
    # set Trigger to HIGH
    GPIO.output(self.GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(self.GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(self.GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(self.GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    
    if(distance < 300):
    	# create a msg of type Float64 (double) to
    	# publish to the /distance topic
    	msg = Float64()
    	msg.data = distance
    
    	# Send distance value to ROS topic
    	self.publisher_.publish(msg)
    
    	#print("Distance: %.4f" % (distance))
  
########### MAIN FUNCTION ###############
# The main function is called automatically when the 
# ping node is launched. It is the entry point 
# for the node

def main(args=None):
    
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node (initializes an instance of the ping class)
    ping = Ping()
    
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(ping)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ping.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
