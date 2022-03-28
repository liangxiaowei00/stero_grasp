# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
from fileinput import filename
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription1 = self.create_subscription(
      Image, 
      '/camera/infra1/image_rect_raw', 
      self.listener_callback1, 
      100)
    self.subscription1 # prevent unused variable warning
      
      
    self.subscription2 = self.create_subscription(
      Image, 
      '/camera/infra2/image_rect_raw', 
      self.listener_callback2, 
      100)
    self.subscription2 # prevent unused variable warning      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.savedir1 = "/home/lxw/cyber_ws/src/Data/infra1/"
    self.savedir2 = "/home/lxw/cyber_ws/src/Data/infra2/"
    self.savenum1 = 0
    self.savenum2 = 0
  def listener_callback1(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame1')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    # cv2.imshow("camera1", current_frame)
    sec = data.header.stamp.sec        
    nanosec = data.header.stamp.nanosec


    filename = self.savedir1 + str(self.savenum1)+"_"+str(sec) +"."+str(nanosec)+ ".bmp"
    self.savenum1 = self.savenum1 + 1
    # print("savenum1 = ",self.savenum1)
    print("filename = ",filename)
    cv2.imwrite(filename,current_frame)
    # cv2.waitKey(1)
    
  
  def listener_callback2(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame2')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    # print("chanal = ",current_frame.shape)

    sec = data.header.stamp.sec        
    nanosec = data.header.stamp.nanosec
    encoding = data.encoding
    # print("encoding = ",encoding)

    # Display image
    # cv2.imshow("camera2", current_frame)
    filename = self.savedir2 + str(self.savenum2)+"_"+str(sec) +"."+str(nanosec)+ ".bmp"
    self.savenum2 = self.savenum2 + 1
    # print("savenum2 = ",self.savenum2)
    print("filename = ",filename)
    cv2.imwrite(filename,current_frame)
    # cv2.waitKey(1)      
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()