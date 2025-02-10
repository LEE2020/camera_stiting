cmake_minimum_required(VERSION 3.13)
project(PanoramicCamera)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -I/usr/local/include/opencv4 -fopenmp")


#set(OpenCV_DIR "/usr/local/lib/cmake/opencv4/OpenCVConfig.cmake")
set(OpenCV_DIR "/home/shenxikeji/opencv_build/opencv/build")
set(OpenCV_INCLUDE_DIRS "/home/shenxikeji/opencv_build/opencv/include")
#set(OpenCV_LIBRARIES "/usr/local/lib/")
#include_directories(${OpenCV_INCLUDE_DIRS})
find_package(OpenCV 4.2.0 REQUIRED)

include_directories("/usr/include/eigen3/")
find_package(Threads)
link_directories("/home/shenxikeji/opencv_build/opencv/build/lib")

add_executable(MTPanoramicCamera multiThread.cpp CameraParameter.cpp Projector.cpp Sticher.cpp MySeamFinder.cpp)
target_link_libraries(MTPanoramicCamera  "${TORCH_LIBRARIES}" ${CMAKE_THREAD_LIBS_INIT} opencv_calib3d opencv_core opencv_dnn opencv_features2d opencv_flann opencv_highgui opencv_imgcodecs opencv_imgproc opencv_ml opencv_objdetect opencv_photo opencv_stitching opencv_video opencv_videoio)
