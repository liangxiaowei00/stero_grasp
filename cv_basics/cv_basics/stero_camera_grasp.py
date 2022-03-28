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
import numpy as np
from skimage import measure,color
 
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
    self.camera_left_points = []
    self.camera_right_points = []
    
    # 重投影矩阵Q
    self.Q = np.array([[1,  0,  0,  -605.679565],
                                    [0,  1,  0,  -544.824665],
                                    [0,  0,  0,  1184.95580],
                                    [0,  0,  8336.24809,  0]])
    
    
  def listener_callback1(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame1')
    self.savenum1 = self.savenum1 + 1
    pixel_dict_left = self.getobjectcentroid(data,self.savenum1)
    self.camera_right_points.append(pixel_dict_left)
    
  
  def listener_callback2(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame2')
    self.savenum2 = self.savenum2 + 1
    pixel_dict_right = self.getobjectcentroid(data,self.savenum2)
    self.camera_left_points.append(pixel_dict_right)
    # print("camera_left_points = ",self.camera_left_points)     
    
  # 通过目标分割提取目标的质心  
  def getobjectcentroid(self,data,frame_id):
    current_frame = self.br.imgmsg_to_cv2(data)
    sec = data.header.stamp.sec        
    nanosec = data.header.stamp.nanosec
    ret, bw_img = cv2.threshold(current_frame, 100, 255, cv2.THRESH_BINARY)
    #cv2.imshow('thresh', bw_img)
    #cv2.waitKey(0)             
    label_image  = measure.label(bw_img)

    max_area=0
    max_area_centroid=None
    max_area_bbox=None
    regions = measure.regionprops(label_image, cache=True)
    for region in regions :
      if(max_area<region.area):
                #print("region.area = ",region.area)
                max_area=region.area
                max_area_centroid=region.centroid
                max_area_bbox=region.bbox
      # print("max_area_centroid = ",max_area_centroid)
      # print("max_area_bbox = ",max_area_bbox)
    #print(f'coast:{time.time() - t:.4f}s')
    #break
  
    pixel_dict ={'frame_id':frame_id,'sec':sec,'nansec':nanosec,'pixel':max_area_centroid}
    print("pixel_dict = ",pixel_dict)
    return pixel_dict
  
  # 像素点到左相机坐标系三维点转换
  def getpoint3d(Q,pixel_left,pixel_right):
    cx = -Q[0][3]
    cy = -Q[1][3]
    f = Q[2][3]
    B = -1/Q[3][2]
    d = pixel_left[0] - pixel_right[0] 
    w = -d/B 
    Z = f / w
    X = (pixel_left[0] + cx)/w
    Y = (pixel_left[1] + cy)/w
    T = np.vstack((X, Y, Z)).T
    print("T= ", T)
    return T
  
  
  
  
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