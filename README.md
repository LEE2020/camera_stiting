# Pedestrian360
The implementation of "Pedestrian-Aware Panoramic Video Stitching Based on a Structured Camera Aray".

## Instructions
* **calibration** : `calibration/calibrate_extrinsics`:  calibrate the extrinsics of the multi-camera system through the images in `calibration/extrinsics_frames`. The results are saved in `calibration/results`
* **video_stitching**: Run `video_stitching/multiThread.cpp` to stitch the videos. 
* **server**: `server/server.py` receive the frames from the stitching program and perform human segmentation using Mask R-CNN. Then it will send the mask data to the stitching program.
* If you want to speed up the stitching, set the rescale factor in `video_stitching/const.h` and `server/server.py` at the same time.
## Requirements

* Python 3.6
  * torch 1.3.0
  * torchvision 0.4.1
  * numpy
  * g2o
  * yaml
  * cv2
  * OpencGL
  * pangolin
* C++
  * cmake
  * OpenCV 4.2.0 
  * eigen3
  * Threads

#opencv安装，调试，编译

# 2.10 
1. 解决图像中的坐标映射问题报错。平面像素坐标转换为三维圆柱体坐标。4个摄像头水平展开，形成一个圆柱体全视野。
#2.11 
1. 复现行人视频拼接项目
#2.12 项目移植
1. 相机标定部分
2. 相机对齐（从4个相机移动到6个相机），相应的处理代码的修改完成
3. 调试
