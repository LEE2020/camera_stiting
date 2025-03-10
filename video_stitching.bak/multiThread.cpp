#include <iostream>
#include <string>
#include <ctime>
#include <thread>
#include <semaphore.h>
#include <fcntl.h>
#include <zconf.h>

#include "opencv2/opencv.hpp"

#include "CameraParameter.h"
#include "Projector.h"
#include "Sticher.h"
#include "Consts.h"

cv::VideoWriter writer;
cv::Mat panoramaResult;
sem_t* semaphore_display;

void stich_frame(int tid){
    cv::VideoCapture cap1, cap2, cap3, cap4,cap5,cap6;   // F, R, B, L
    //cap1.open("./resources/videos/F500_0903_4.avi");
    //cap2.open("./resources/videos/R500_0903_4.avi");
    //cap3.open("./resources/videos/B500_0903_4.avi");
    //cap4.open("./resources/videos/L500_0903_4.avi");
    cap1.open("../camera_ff_1.avi");
    cap2.open("../camera_fl_6.avi");
    cap3.open("../camera_bl_3.avi");
    cap4.open("../camera_bb_2.avi");
    cap5.open("../camera_br_4.avi");
    cap6.open("../camera_fr_5.avi");

    // 读取一帧并显示各视频流的图像
    cv::Mat frameFF, frameFL, frameBL,frameBB,frameBR, frameFR;
    cap1 >> frameFF;
    cap2 >> frameFL;
    cap3 >> frameBL;
    cap4 >> frameBB;
    cap5 >> frameBR;
    cap6 >> frameFR;

    pc::Sticher sticher;
    std::vector<cv::Mat> imgs(pc::numCamera);
    std::vector<cv::UMat> remapImgs;
    for(int i=0;i<pc::numCamera;i++) {
        remapImgs.push_back(cv::UMat(pc::stitchResultSize, CV_8U));
    }
    cv::Mat temp_panoramaResult;

    int temp=200;
    while(temp--){
        cap1>>imgs[0];
        cap2>>imgs[1];
        cap3>>imgs[2];
        cap4>>imgs[3];
        cap5>>imgs[4];
        cap6>>imgs[5];

        sticher.stich(panoramaResult, imgs, remapImgs);
        sem_post(semaphore_display);
        clock_t time_end2=std::clock();
    }
    return;
}

int main() {

    semaphore_display = sem_open("sem1",O_CREAT, S_IRUSR | S_IWUSR, 0);
    //采用MJPG编码，文件名为result.avi，帧率为5，画面尺寸为pc::stitchResultSize
    writer.open("../result.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 5, pc::stitchResultSize, true);

    // 创建显示窗口
    cv::namedWindow("result", cv::WINDOW_AUTOSIZE);

    std::thread threads[pc::threads_num];
    for(int i=0; i<pc::threads_num; i++){
        threads[i] = std::thread(std::ref(stich_frame), i);
    }

    int temp=200;
    while(temp--){
        sem_wait(semaphore_display);
        // 检查图像是否有效再显示
        if (!panoramaResult.empty()){
            cv::imshow("result", panoramaResult);
            writer.write(panoramaResult);
        } else {
            std::cerr << "Warning: panoramaResult is empty!" << std::endl;
        }
        int key = cv::waitKey(10);
        if(key>0)
            break;
    }

    writer.release();

    for(int i = 0; i < pc::threads_num; i++){
        if(threads[i].joinable())
            threads[i].join();
    }

    sem_close(semaphore_display);
    sem_unlink("sem1");
}