
from copy import deepcopy
import cv2
import numpy as np
import stereoconfig   #导入相机标定的参数

# 获取畸变校正和立体校正的映射变换矩阵、重投影矩阵
# @param：config是一个类，存储着双目标定的参数:config = stereoconfig.stereoCamera()
def getRectifyTransform(height, width, config):
    # 读取内参和外参
    left_K = config.cam_matrix_left
    right_K = config.cam_matrix_right
    left_distortion = config.distortion_l
    right_distortion = config.distortion_r
    R = config.R
    T = config.T

    # 计算校正变换
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(left_K, left_distortion, right_K, right_distortion, 
                                                    (width, height), R, T, alpha=0)

    map1x, map1y = cv2.initUndistortRectifyMap(left_K, left_distortion, R1, P1, (width, height), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(right_K, right_distortion, R2, P2, (width, height), cv2.CV_32FC1)

    return map1x, map1y, map2x, map2y, Q

if __name__ == '__main__':

    # 读取相机内参和外参
    config = stereoconfig.stereoCamera()

    # 立体校正
    map1x, map1y, map2x, map2y, Q = getRectifyTransform(1280, 1024, config)  # 获取用于畸变校正和立体校正的映射矩阵以及用于计算像素空间坐标的重投影矩阵
    print("Print Q!")
    print(Q)
    # 以左相机为坐标原点的坐标系为基准
    # 待测像素点对左相机（x1,y1）右相机（x2,y2）对应的三维坐标（X，Y，Z）
    


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
  
        
   
    
    
