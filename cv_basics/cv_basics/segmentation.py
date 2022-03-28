
from fileinput import filename
import cv2 # OpenCV library
import os
import numpy as np
from skimage import measure,color
import time
import pickle





def save_variable(v,filename):
    f=open(filename,'wb')
    pickle.dump(v,f)
    f.close()
    return filename
 
def load_variavle(filename):
    f=open(filename,'rb')
    r=pickle.load(f)
    f.close()
    return r



def main(args=None):
  # data=load_variavle('test.txt')
  # print(data)
  in_dir="/home/lxw/cyber_ws/src/Data/infra2_seg/"
  out_dir = "/home/lxw/cyber_ws/src/Data/infra2_seg_out/"
  if not os.path.exists(out_dir):
            os.makedirs(out_dir)
  for root ,dirs,files, in os.walk(in_dir,topdown=False):
            # print(files)
            pass
  pixel_array=[]
  for file in files:
    #file = "127_1648092392.822649600.bmp"
    current_frame = cv2.imread(in_dir+file,flags=0)
    print("file = ",file)
    #cv2.imshow("camera", current_frame)
    #cv2.waitKey(0)   

    #t = time.time()
    #for i in range(1000):
              
    pixel_dict=getobjectcentroid(current_frame)
    pixel_array.append(pixel_dict);
    rgb_image = cv2.cvtColor(current_frame,cv2.COLOR_GRAY2BGR)
    #if len(regions)>0:
        #cv2.rectangle(rgb_image, (max_area_bbox[1], max_area_bbox[0]), (max_area_bbox[3], max_area_bbox[2]), (0, 0, 255), thickness=2)
    cv2.imshow("camera_RGB", rgb_image)
    filename = out_dir+file
    #cv2.imwrite(filename,rgb_image)
    '''
    filename = self.savedir1 + str(sec) +"."+str(nanosec)+ ".bmp"
    self.savenum1 = self.savenum1 + 1
    print("savenum1 = ",self.savenum1)
    print("filename = ",filename)
    
    cv2.imwrite(filename,current_frame)
    '''
    cv2.waitKey(1)   
  #print(pixel_array)
  #filename=save_variable(pixel_array,'test.txt')

def getobjectcentroid(current_frame):
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
  
    #pixel_dict ={'frame_num':1,'sec':1,'nansec':1,'pixel':max_area_centroid}
   # print("pixel_dict = ",pixel_dict)
    print("max_area_centroid = ",max_area_centroid)

    return max_area_centroid
    
            
  
if __name__ == '__main__':
  main()