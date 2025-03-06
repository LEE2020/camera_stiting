#ifndef PANORAMICCAMERA_STICHER_H
#define PANORAMICCAMERA_STICHER_H


#include <iostream>
#include <ctime>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "CameraParameter.h"
#include "Projector.h"
#include "MySeamFinder.h"
#include "Consts.h"

namespace pc{
    class Sticher {
    public:
        Sticher();

        std::vector<CylinderProjector> projectors;
        std::vector<cv::Mat> xMaps, yMaps;
        std::vector<cv::UMat> maskMaps;
        std::vector<cv::UMat> resizeMaskMaps;
        std::vector<cv::UMat> resizeMaskMapsSeam;
        std::vector<cv::UMat> maskMapsSeam;
        std::vector<cv::Point> corners;
        cv::Mat overlapFR, overlapRB, overlapBL, overlapLF;
        //std::vector<Eigen::Vector4d> photometricAlignment_parameters; //3 channels * 4 images
        std::vector<Eigen::Matrix<double,6,1>> photometricAlignment_parameters; // 3 channels * 6 images
        MySeamFinder * mySeamFinder;
        cv::Mat overlapFF_FL,overlapFL_BL,overlapBL_BB,overlapBB_BR,overlapBR_FR,overlapFR_FF;
        

        cv::Ptr<cv::detail::SeamFinder> seamFinderPtr;
        cv::Ptr<cv::detail::Blender> blenderPtr;

        void photometricAlignment_std(cv::UMat &imgF, cv::UMat &imgR, cv::UMat &imgB, cv::UMat &imgL);
        void compute_photometricAlignment_std(cv::UMat &imgF, cv::UMat &imgR, cv::UMat &imgB, cv::UMat &imgL);
        void photometricAlignment_std_new(cv::UMat &imgFF, cv::UMat &imgFL, cv::UMat &imgBL, cv::UMat &imgBB,cv::UMat &imgBR,cv::UMat &imgFR);
        void photometricAlignment_std2(cv::UMat &imgFF, cv::UMat &imgFL, cv::UMat &imgBL, cv::UMat &imgBB,cv::UMat &imgBR,cv::UMat &imgFR);
        void compute_photometricAlignment_std2(cv::UMat &imgFF, cv::UMat &imgFL, cv::UMat &imgBL,cv::UMat &imgBB,cv::UMat &imgBR,cv::UMat &imgFR);

        void stich(cv::Mat& result, const std::vector<cv::Mat>& imgs, std::vector<cv::UMat>& remapImgs,
                int blend_type=cv::detail::Blender::FEATHER);

        void stich_ori(cv::Mat& result, const std::vector<cv::Mat>& imgs, std::vector<cv::UMat>& remapImgs,
                   int seam_type=cv::detail::SeamFinder::NO,
                   int blend_type=cv::detail::Blender::FEATHER);


    };
}



#endif //PANORAMICCAMERA_STICHER_H