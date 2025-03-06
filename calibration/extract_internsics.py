
import numpy as np
data_ff = np.load('./intrinsics_data/camera_calib_1.npz')
data_bb = np.load('./intrinsics_data/camera_calib_2.npz')
data_bl = np.load('./intrinsics_data/camera_calib_3.npz')
data_br = np.load('./intrinsics_data/camera_calib_4.npz')
data_fr = np.load('./intrinsics_data/camera_calib_5.npz')
data_fl = np.load('./intrinsics_data/camera_calib_6.npz')

#内参数矩阵
K1 = data_fr['camera_matrix']
D1 = data_fr['dist_coeffs']
fx =K1[0,0]
fy = K1[1,1]
cx = K1[0,2]
cy = K1[1,2]
k1 = D1[0]
print('fx:',fx,'fy:',fy,'cx:',cx,'cy：',cy,'K1：',k1)

a=[[3.46372585e+03, 0.00000000e+00, 1.07141798e+03]
, [0.00000000e+00, 3.23165370e+03, 3.33628077e+02]
,[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]

fx =a[0][0]
fy = a[1][1]
cx = a[0][2]
cy = a[1][2]
print('fx:',fx,'fy:',fy,'cx:',cx,'cy：',cy)