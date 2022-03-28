import numpy as np


####################仅仅是一个示例###################################


# 双目相机参数
class stereoCamera(object):
    def __init__(self):
        # 左相机内参
        self.cam_matrix_left = np.array([   [830.5873,   -3.0662,  658.1007],
                                            [       0,  830.8116,  482.9859],
                                            [       0,         0,         1]
                                        ])
        # 右相机内参
        self.cam_matrix_right = np.array([  [830.4255,   -3.5852,  636.8418],
                                            [       0,  830.7571,  476.0664],
                                            [       0,         0,         1]
                                        ])

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        self.distortion_l = np.array([[-0.0806, 0.3806, -0.0033, 0.0005148]])
        self.distortion_r = np.array([[-0.0485, 0.2200, -0.002,  0.0017]])

        # 旋转矩阵
        self.R = np.array([ [      1,  0.0017, -0.0093],
                            [-0.0018,  1.0000, -0.0019],
                            [ 0.0093,  0.0019,  1.0000]   
                            ])

        # 平移矩阵
        self.T = np.array([[-119.9578], [0.1121], [-0.2134]])


        
        # 重投影矩阵Q
        self.Q = np.array([[1,  0,  0,  -605.679565],
                                        [0,  1,  0,  -544.824665],
                                        [0,  0,  0,  1184.95580],
                                        [0,  0,  8336.24809,  0]])
                
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
