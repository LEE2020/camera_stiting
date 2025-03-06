#include "Sticher.h"
#include "MySeamFinder.h"
#include <string>
#include "Projector.h"

pc::Sticher::Sticher(){
    // init
    std::vector<pc::CameraParameter> cameras;
    for(int i=0;i<pc::numCamera;i++){
        pc::CameraParameter camera = (std::string(pc::parm_files[i]));
        cameras.push_back(camera);
        projectors.push_back(pc::CylinderProjector(camera, 24/pc::disFloorToCenter));
        xMaps.push_back(cv::Mat(pc::stitchResultSize, CV_32F));
        yMaps.push_back(cv::Mat(pc::stitchResultSize, CV_32F));
        maskMaps.push_back(cv::UMat::zeros(pc::stitchResultSize, CV_8U));
        maskMapsSeam.push_back(cv::UMat::zeros(pc::stitchResultSize, CV_8U));
        resizeMaskMapsSeam.push_back(cv::UMat::zeros(pc::resizeStitchResultSize, CV_8U));
        resizeMaskMaps.push_back(cv::UMat(pc::resizeStitchResultSize, CV_8U));
        corners.push_back(cv::Point(0,0));
    }

    // compute xmap ymap and mask
    double x,y;
    for(int k=0;k<pc::numCamera;k++){
        cv::Mat maskMaps_temp = maskMaps[k].getMat(cv::ACCESS_WRITE);
        for(int i=0; i<pc::stitchResultSize.width; i++){
            for(int j=0; j<pc::stitchResultSize.height; j++){
                // 添加校验
                if(projectors[k].mapBackward(i, j, x, y) &&
                   x>=0 && x<pc::photoSize.width && y>=0 && y<pc::photoSize.height){
                    xMaps[k].at<float>(j,i) = static_cast<float>(x);
                    yMaps[k].at<float>(j,i) = static_cast<float>(y);
                    maskMaps_temp.at<uchar>(j,i) = 255;
                } else {
                    // 如果计算无效，可选择赋予默认值或者打印警告
                    // xMaps[k].at<float>(j,i) = -1;
                    // yMaps[k].at<float>(j,i) = -1;
                }
            }
        }
        // 恢复调试输出打印
        //double minVal, maxVal;
        //cv::minMaxLoc(xMaps[k], &minVal, &maxVal);
       // std::cout << "Camera " << k << " xMap: min=" << minVal << " max=" << maxVal << std::endl;
       // cv::minMaxLoc(yMaps[k], &minVal, &maxVal);
      //  std::cout << "Camera " << k << " yMap: min=" << minVal << " max=" << maxVal << std::endl;
    }

    //cv::bitwise_and(maskMaps[0], maskMaps[1], overlapFR, maskMaps[0]);
    //cv::bitwise_and(maskMaps[1], maskMaps[2], overlapRB, maskMaps[1]);
    //cv::bitwise_and(maskMaps[2], maskMaps[3], overlapBL, maskMaps[2]);
    //cv::bitwise_and(maskMaps[3], maskMaps[0], overlapLF, maskMaps[3]);
    overlapFF_FL = cv::Mat(pc::stitchResultSize, CV_8U, cv::Scalar(255));
    overlapFL_BL = cv::Mat(pc::stitchResultSize, CV_8U, cv::Scalar(255));
    overlapBL_BB = cv::Mat(pc::stitchResultSize, CV_8U, cv::Scalar(255));
    overlapBB_BR = cv::Mat(pc::stitchResultSize, CV_8U, cv::Scalar(255));
    overlapBR_FR = cv::Mat(pc::stitchResultSize, CV_8U, cv::Scalar(255));
    overlapFR_FF = cv::Mat(pc::stitchResultSize, CV_8U, cv::Scalar(255));

    cv::bitwise_and(maskMaps[0], maskMaps[1], overlapFF_FL, maskMaps[0]);
    cv::bitwise_and(maskMaps[1], maskMaps[2], overlapFL_BL, maskMaps[1]);
    cv::bitwise_and(maskMaps[2], maskMaps[3], overlapBL_BB, maskMaps[2]);
    cv::bitwise_and(maskMaps[3], maskMaps[4], overlapBB_BR, maskMaps[3]);
    cv::bitwise_and(maskMaps[4], maskMaps[5], overlapBR_FR, maskMaps[4]);   
    cv::bitwise_and(maskMaps[5], maskMaps[0], overlapFR_FF, maskMaps[5]);
    

    // init seam
    for(int i=0;i<pc::numCamera;i++) {
        cv::resize(maskMaps[i], resizeMaskMaps[i], pc::resizeStitchResultSize);
    }

    mySeamFinder = new pc::MySeamFinder(resizeMaskMaps);

    for(int i=0; i<3; i++)
        //photometricAlignment_parameters.push_back(Eigen::Vector4d(1.0 , 1.0 , 1.0 , 1.0));
        photometricAlignment_parameters.push_back(Eigen::Matrix<double,6,1>::Ones());
}


void pc::Sticher::photometricAlignment_std_new(cv::UMat &imgFF, cv::UMat &imgFL,cv::UMat &imgBL, cv::UMat &imgBB,
    cv::UMat &imgBR, cv::UMat &imgFR ){
    
    //cv::Scalar FF_FL_mean({0,0,0,0,0,0});
    //cv::Scalar FL_BL_mean({0,0,0,0,0,0});
    //cv::Scalar BL_BB_mean({0,0,0,0,0,0});
   // cv::Scalar BB_BR_mean({0,0,0,0,0,0}); 
   // cv::Scalar BR_FR_mean({0,0,0,0,0,0});
    //cv::Scalar FR_FF_mean({0,0,0,0,0,0});

    cv::Mat FF_YCbCr, FL_YCbCr, BL_YCbCr,BB_YCbCr,BR_YCbCr, FR_YCbCr;

    std::vector<cv::Mat> FF_channels, FL_channels,BL_channels, BB_channels, BR_channels, FR_channels;
    cv::Scalar meanFFFL_FF, meanFFFL_FL, meanFLBL_FL, meanFLBL_BL, meanBLBB_BL, meanBLBB_BB,\
     meanBBBR_BB, meanBBBR_BR, meanBRFR_BR, meanBRFR_FR, meanFRFF_FR, meanFRFF_FF;

    cv::Scalar stdFFFL_FF, stdFFFL_FL, stdFLBL_FL, stdFLBL_BL, stdBLBB_BL, stdBLBB_BB,\
     stdBBBR_BB, stdBBBR_BR, stdBRFR_BR, stdBRFR_FR, stdFRFF_FR, stdFRFF_FF;
    omp_set_num_threads(6);
#pragma omp parallel for
for(int i=0; i<6; i++){
    switch (i) {
        case 0:
            cv::cvtColor(imgFF, FF_YCbCr, cv::COLOR_BGR2YCrCb);
            cv::split(FF_YCbCr, FF_channels);
            cv::meanStdDev(FF_YCbCr, meanFFFL_FF, stdFFFL_FF, overlapFF_FL);
            cv::meanStdDev(FF_YCbCr, meanFRFF_FF, stdFRFF_FF, overlapFR_FF);
            break;
        case 1:
            cv::cvtColor(imgFL, FL_YCbCr, cv::COLOR_BGR2YCrCb);
            cv::split(FL_YCbCr, FL_channels);
            cv::meanStdDev(FL_YCbCr, meanFFFL_FL, stdFFFL_FL, overlapFF_FL);
            cv::meanStdDev(FL_YCbCr, meanFLBL_FL, stdFLBL_FL, overlapFL_BL);
            break;
        case 2:
            cv::cvtColor(imgBL, BL_YCbCr, cv::COLOR_BGR2YCrCb);
            cv::split(BL_YCbCr, BL_channels);
            cv::meanStdDev(BL_YCbCr, meanFLBL_BL, stdFLBL_BL, overlapFL_BL);
            cv::meanStdDev(BL_YCbCr, meanBLBB_BL, stdBLBB_BL, overlapBL_BB);
            break;
            
        case 3:
            cv::cvtColor(imgBB, BB_YCbCr, cv::COLOR_BGR2YCrCb);
            cv::split(BB_YCbCr, BB_channels);
            cv::meanStdDev(BB_YCbCr, meanBLBB_BB, stdBLBB_BB, overlapBL_BB);
            cv::meanStdDev(BB_YCbCr, meanBBBR_BB, stdBBBR_BB, overlapBB_BR);
            break;
        case 4:
            cv::cvtColor(imgBR, BR_YCbCr, cv::COLOR_BGR2YCrCb);
            cv::split(BR_YCbCr, BR_channels);
            cv::meanStdDev(BR_YCbCr, meanBBBR_BR, stdBBBR_BR, overlapBB_BR);
            cv::meanStdDev(BR_YCbCr, meanBRFR_BR, stdBRFR_BR, overlapBR_FR);
            break;
        case 5:
            cv::cvtColor(imgFR, FR_YCbCr, cv::COLOR_BGR2YCrCb);
            cv::split(FR_YCbCr, FR_channels);
            cv::meanStdDev(FR_YCbCr, meanBRFR_FR, stdBRFR_FR, overlapBR_FR);
            cv::meanStdDev(FR_YCbCr, meanFRFF_FR, stdFRFF_FR, overlapFR_FF);
    }
}
omp_set_num_threads(3);
//#pragma omp parallel for
    for(int iChannel = 0; iChannel<3; iChannel++){
        //Eigen::Matrix4d iA;
        Eigen::Matrix<double, 6, 6> iA;
        iA.setZero();
        // Row0: overlapFF_FL (图像FF与FL)
        iA(0, 0) = stdFFFL_FF[iChannel];
        iA(0, 1) = -stdFFFL_FL[iChannel];
    // Row1: overlapFL_BL (图像FL与BL)
        iA(1, 1) = stdFLBL_FL[iChannel];
        iA(1, 2) = -stdFLBL_BL[iChannel];
    // Row2: overlapBL_BB (图像BL与BB)
        iA(2, 2) = stdBLBB_BB[iChannel];
        iA(2, 3) = -stdBLBB_BL[iChannel];
    // Row3: overlapBB_BR (图像BB与BR)
        iA(3, 3) = stdBBBR_BB[iChannel];
        iA(3, 4) = -stdBBBR_BR[iChannel];
    // Row4: overlapBR_FR (图像BR与FR)
        iA(4, 4) = stdBRFR_BR[iChannel];
        iA(4, 5) = -stdBRFR_FR[iChannel];
    // Row5: overlapFR_FF (图像FR与FF)
        iA(5, 5) = stdFRFF_FR[iChannel];
        iA(5, 0) = -stdFRFF_FF[iChannel];
        Eigen::EigenSolver<Eigen::Matrix<double, 6, 6>> solver(iA);
        Eigen::VectorXd values = solver.eigenvalues().real();
        Eigen::MatrixXd vectors = solver.eigenvectors().real();
        double minValue = values[0];
        int minPos = 0;


        for (int i=1;i<6;i++){
            if (values[i] < minValue){
                minValue = values[i];
                minPos = i;
            }
        }
        Eigen::VectorXd iNewG = Eigen::VectorXd::Zero(6); // 初始化为 6x1 的零向量
        iNewG = vectors.block<6,1>(0 , minPos);
        //Eigen::VectorXd iNewG(1.0 , 1.0 , 1.0 , 1.0, 1.0, 1.0);
        //iNewG = vectors.block<6,1>(0 , minPos);
//        double min = iNewG[0];
//        for (int i=1;i<4;i++){
//            if (min * min < iNewG[i] * iNewG[i]){
//                min = iNewG[i];
//            }
//        }
        double mean = iNewG.mean();
//        std::cout<<iNewG<<std::endl;
//        std::cout<<min<<std::endl;
        iNewG /= mean;
//        std::cout<<iNewG<<std::endl;
        // 构造 6x6 差分矩阵 A2，用以建立偏移的线性关系
        Eigen::Matrix<double, 6, 6> iA2;
        iA2.setZero();
        iA2(0, 0) = 1;  iA2(0, 1) = -1;
        iA2(1, 1) = 1;  iA2(1, 2) = -1;
        iA2(2, 2) = 1;  iA2(2, 3) = -1;
        iA2(3, 3) = 1;  iA2(3, 4) = -1;
        iA2(4, 4) = 1;  iA2(4, 5) = -1;
        iA2(5, 5) = 1;  iA2(5, 0) = -1;

// 构造 6x1 向量 ib2，对应每个重叠区域两侧均值的关系
        
      
        Eigen::Matrix<double, 6, 1> ib2;
        ib2(0) = -meanFFFL_FF[iChannel] * iNewG(0) + meanFFFL_FL[iChannel] * iNewG(1);
        ib2(1) = -meanFLBL_FL[iChannel] * iNewG(1) + meanFLBL_BL[iChannel] * iNewG(2);
        ib2(2) = -meanBLBB_BL[iChannel] * iNewG(2) + meanBLBB_BB[iChannel] * iNewG(3);
        ib2(3) = -meanBBBR_BB[iChannel] * iNewG(3) + meanBBBR_BR[iChannel] * iNewG(4);
        ib2(4) = -meanBRFR_BR[iChannel] * iNewG(4) + meanBRFR_FR[iChannel] * iNewG(5);
        ib2(5) = -meanFRFF_FF[iChannel] * iNewG(5) + meanFRFF_FR[iChannel] * iNewG(0);
        Eigen::VectorXd b = iA2.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(ib2);

        

//        std::cout<<b<<std::endl;
        FF_channels[iChannel] = FF_channels[iChannel]*iNewG[0] + b[0];
        FL_channels[iChannel] = FL_channels[iChannel]*iNewG[1] + b[1];
        BL_channels[iChannel] = BL_channels[iChannel]*iNewG[2] + b[2];
        BB_channels[iChannel] = BB_channels[iChannel]*iNewG[3] + b[3];
        BR_channels[iChannel] = BR_channels[iChannel]*iNewG[4] + b[4];
        FR_channels[iChannel] = FR_channels[iChannel]*iNewG[5] + b[5];

    }
    cv::merge(FF_channels, FF_YCbCr);
    cv::merge(FL_channels, FL_YCbCr);
    cv::merge(BL_channels, BL_YCbCr);
    cv::merge(BB_channels, BB_YCbCr);
    cv::merge(BR_channels, BR_YCbCr);
    cv::merge(FR_channels, FR_YCbCr);

    cv::cvtColor(FF_YCbCr, imgFF, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(FL_YCbCr, imgFL, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(BL_YCbCr, imgBL, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(BB_YCbCr, imgBB, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(BR_YCbCr, imgBR, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(FR_YCbCr, imgFR, cv::COLOR_YCrCb2BGR);




    }

void pc::Sticher::photometricAlignment_std(cv::UMat &imgF, cv::UMat &imgR, cv::UMat &imgB, cv::UMat &imgL) {

//    cv::Mat F_YCbCr, L_YCbCr, B_YCbCr, R_YCbCr;
//    cv::cvtColor(imgF, F_YCbCr, cv::COLOR_BGR2YCrCb);
//    cv::cvtColor(imgR, R_YCbCr, cv::COLOR_BGR2YCrCb);
//    cv::cvtColor(imgB, B_YCbCr, cv::COLOR_BGR2YCrCb);
//    cv::cvtColor(imgL, L_YCbCr, cv::COLOR_BGR2YCrCb);
//    std::vector<cv::Mat> F_channels, L_channels, B_channels, R_channels;
//    cv::split(F_YCbCr, F_channels);
//    cv::split(R_YCbCr, R_channels);
//    cv::split(B_YCbCr, B_channels);
//    cv::split(L_YCbCr, L_channels);
//
//    cv::Scalar meanFR_F, meanFR_R, meanRB_R, meanRB_B, meanBL_B, meanBL_L, meanLF_L, meanLF_F;
//    cv::Scalar stdFR_F, stdFR_R, stdRB_R, stdRB_B, stdBL_B, stdBL_L, stdLF_L, stdLF_F;
//    cv::meanStdDev(F_YCbCr, meanFR_F, stdFR_F, overlapFR);
//    cv::meanStdDev(R_YCbCr, meanFR_R, stdFR_R, overlapFR);
//    cv::meanStdDev(R_YCbCr, meanRB_R, stdRB_R, overlapRB);
//    cv::meanStdDev(B_YCbCr, meanRB_B, stdRB_B, overlapRB);
//    cv::meanStdDev(B_YCbCr, meanBL_B, stdBL_B, overlapBL);
//    cv::meanStdDev(L_YCbCr, meanBL_L, stdBL_L, overlapBL);
//    cv::mea
cv::Scalar FR_mean({0,0,0,0});
cv::Scalar RB_mean({0,0,0,0});
cv::Scalar BL_mean({0,0,0,0});
cv::Scalar LF_mean({0,0,0,0}); 
//    cv::meanStdDev(F_YCbCr, meanLF_F, stdLF_F, overlapLF);

    cv::Mat F_YCbCr, L_YCbCr, B_YCbCr, R_YCbCr;
    std::vector<cv::Mat> F_channels, L_channels, B_channels, R_channels;
    cv::Scalar meanFR_F, meanFR_R, meanRB_R, meanRB_B, meanBL_B, meanBL_L, meanLF_L, meanLF_F;
    cv::Scalar stdFR_F, stdFR_R, stdRB_R, stdRB_B, stdBL_B, stdBL_L, stdLF_L, stdLF_F;
   // omp_set_num_threads(4);
//#pragma omp parallel for
    for(int i=0; i<4; i++){
        switch (i) {
            case 0:
                cv::cvtColor(imgF, F_YCbCr, cv::COLOR_BGR2YCrCb);
                cv::split(F_YCbCr, F_channels);
                cv::meanStdDev(F_YCbCr, meanFR_F, stdFR_F, overlapFR);
                cv::meanStdDev(F_YCbCr, meanLF_F, stdLF_F, overlapLF);
                break;
            case 1:
                cv::cvtColor(imgR, R_YCbCr, cv::COLOR_BGR2YCrCb);
                cv::split(R_YCbCr, R_channels);
                cv::meanStdDev(R_YCbCr, meanFR_R, stdFR_R, overlapFR);
                cv::meanStdDev(R_YCbCr, meanRB_R, stdRB_R, overlapRB);
                break;
            case 2:
                cv::cvtColor(imgB, B_YCbCr, cv::COLOR_BGR2YCrCb);
                cv::split(B_YCbCr, B_channels);
                cv::meanStdDev(B_YCbCr, meanRB_B, stdRB_B, overlapRB);
                cv::meanStdDev(B_YCbCr, meanBL_B, stdBL_B, overlapBL);
                break;
            case 3:
                cv::cvtColor(imgL, L_YCbCr, cv::COLOR_BGR2YCrCb);
                cv::split(L_YCbCr, L_channels);
                cv::meanStdDev(L_YCbCr, meanBL_L, stdBL_L, overlapBL);
                cv::meanStdDev(L_YCbCr, meanLF_L, stdLF_L, overlapLF);
        }
    }

    omp_set_num_threads(3);
//#pragma omp parallel for
    for(int iChannel = 0; iChannel<3; iChannel++){
        Eigen::Matrix4d iA;
        iA << 	stdFR_F[iChannel] , -stdFR_R[iChannel] ,	 0.0  			,	 	0.0		,
                0.0 		 , stdRB_R[iChannel]  , 	 -stdRB_B[iChannel] 	,		0.0		,
                0.0			 , 0.0			 ,   stdBL_B[iChannel] 	,  -stdBL_L[iChannel],
                -stdLF_L[iChannel], 0.0			 , 	0.0				,  stdLF_F[iChannel] ;
        Eigen::EigenSolver<Eigen::Matrix4d> solver(iA);
        Eigen::Vector4d values = solver.eigenvalues().real();
        Eigen::Matrix4d vectors = solver.eigenvectors().real();
        double minValue = values[0];
        int minPos = 0;
        for (int i=1;i<4;i++){
            if (values[i] < minValue){
                minValue = values[i];
                minPos = i;
            }
        }
        Eigen::Vector4d iNewG(1.0 , 1.0 , 1.0 , 1.0);
        iNewG = vectors.block<4,1>(0 , minPos);
//        double min = iNewG[0];
//        for (int i=1;i<4;i++){
//            if (min * min < iNewG[i] * iNewG[i]){
//                min = iNewG[i];
//            }
//        }
        double mean = iNewG.mean();
//        std::cout<<iNewG<<std::endl;
//        std::cout<<min<<std::endl;
        iNewG /= mean;
//        std::cout<<iNewG<<std::endl;

        Eigen::Matrix4d iA2;
        iA2 << 	1 , -1 ,	 0.0  			,	 	0.0		,
                0.0 		 , 1  , 	 -1 	,		0.0		,
                0.0			 , 0.0			 ,   1	,  -1,
                -1, 0.0			 , 	0.0				,  1 ;
        Eigen::Vector4d ib2;
        ib2 << -meanFR_F[iChannel]*iNewG[0] + meanFR_R[iChannel]*iNewG[1],
                -meanRB_R[iChannel]*iNewG[1] + meanRB_B[iChannel]*iNewG[2],
                -meanBL_B[iChannel]*iNewG[2] + meanBL_L[iChannel]*iNewG[3],
                -meanLF_L[iChannel]*iNewG[3] + meanLF_F[iChannel]*iNewG[0];

        Eigen::Vector4d b = iA2.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(ib2);

//        std::cout<<b<<std::endl;

        F_channels[iChannel] = F_channels[iChannel]*iNewG[0] + b[0];
        R_channels[iChannel] = R_channels[iChannel]*iNewG[1] + b[1];
        B_channels[iChannel] = B_channels[iChannel]*iNewG[2] + b[2];
        L_channels[iChannel] = L_channels[iChannel]*iNewG[3] + b[3];

    }

    cv::merge(F_channels, F_YCbCr);
    cv::merge(L_channels, L_YCbCr);
    cv::merge(B_channels, B_YCbCr);
    cv::merge(R_channels, R_YCbCr);

    cv::cvtColor(F_YCbCr, imgF, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(L_YCbCr, imgL, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(B_YCbCr, imgB, cv::COLOR_YCrCb2BGR);
    cv::cvtColor(R_YCbCr, imgR, cv::COLOR_YCrCb2BGR);
}

cv::Scalar FR_mean({0,0,0,0});
cv::Scalar RB_mean({0,0,0,0});
cv::Scalar BL_mean({0,0,0,0});
cv::Scalar LF_mean({0,0,0,0});

cv::Scalar scalarabs(cv::Scalar sc){
    cv::Scalar res(sc);
    for(int i=0; i<3; i++){
        res[i] = std::abs(sc[i]);
    }
    return res;
}

void pc::Sticher::compute_photometricAlignment_std(cv::UMat &imgF, cv::UMat &imgR, cv::UMat &imgB, cv::UMat &imgL){
    cv::Mat F_YCbCr, L_YCbCr, B_YCbCr, R_YCbCr;
    cv::cvtColor(imgF, F_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgR, R_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgB, B_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgL, L_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::Scalar meanFR_F, meanFR_R, meanRB_R, meanRB_B, meanBL_B, meanBL_L, meanLF_L, meanLF_F;
    cv::Scalar stdFR_F, stdFR_R, stdRB_R, stdRB_B, stdBL_B, stdBL_L, stdLF_L, stdLF_F;
    cv::meanStdDev(F_YCbCr, meanFR_F, stdFR_F, overlapFR);
    cv::meanStdDev(R_YCbCr, meanFR_R, stdFR_R, overlapFR);
    cv::meanStdDev(R_YCbCr, meanRB_R, stdRB_R, overlapRB);
    cv::meanStdDev(B_YCbCr, meanRB_B, stdRB_B, overlapRB);
    cv::meanStdDev(B_YCbCr, meanBL_B, stdBL_B, overlapBL);
    cv::meanStdDev(L_YCbCr, meanBL_L, stdBL_L, overlapBL);
    cv::meanStdDev(L_YCbCr, meanLF_L, stdLF_L, overlapLF);
    cv::meanStdDev(F_YCbCr, meanLF_F, stdLF_F, overlapLF);

    FR_mean += scalarabs(meanFR_F-meanFR_R);
    RB_mean += scalarabs(meanRB_R-meanRB_B);
    BL_mean += scalarabs(meanBL_B-meanBL_L);
    LF_mean += scalarabs(meanLF_L-meanLF_F);

}
void pc::Sticher::compute_photometricAlignment_std2(cv::UMat &imgFF, cv::UMat &imgFL,
                                                     cv::UMat &imgBL, cv::UMat &imgBB,
                                                     cv::UMat &imgBR, cv::UMat &imgFR) {
    // 将6幅图像从BGR转换为YCrCb空间
    cv::Mat FF_YCbCr, FL_YCbCr, BL_YCbCr, BB_YCbCr, BR_YCbCr, FR_YCbCr;
    cv::cvtColor(imgFF, FF_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgFL, FL_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgBL, BL_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgBB, BB_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgBR, BR_YCbCr, cv::COLOR_BGR2YCrCb);
    cv::cvtColor(imgFR, FR_YCbCr, cv::COLOR_BGR2YCrCb);

    // 利用预定义的6个重叠区域掩码，计算各区域的均值与标准差
    // 0: overlapFF_FL (图像FF与FL)
    cv::Scalar meanFF_FL, stdFF_FL;
    cv::meanStdDev(FF_YCbCr, meanFF_FL, stdFF_FL, overlapFF_FL);
    cv::Scalar meanFL_FF, stdFL_FF;
    cv::meanStdDev(FL_YCbCr, meanFL_FF, stdFL_FF, overlapFF_FL);

    // 1: overlapFL_BL (图像FL与BL)
    cv::Scalar meanFL_BL, stdFL_BL;
    cv::meanStdDev(FL_YCbCr, meanFL_BL, stdFL_BL, overlapFL_BL);
    cv::Scalar meanBL_FL, stdBL_FL;
    cv::meanStdDev(BL_YCbCr, meanBL_FL, stdBL_FL, overlapFL_BL);

    // 2: overlapBL_BB (图像BL与BB)
    cv::Scalar meanBL_BB, stdBL_BB;
    cv::meanStdDev(BL_YCbCr, meanBL_BB, stdBL_BB, overlapBL_BB);
    cv::Scalar meanBB_BL, stdBB_BL;
    cv::meanStdDev(BB_YCbCr, meanBB_BL, stdBB_BL, overlapBL_BB);

    // 3: overlapBB_BR (图像BB与BR)
    cv::Scalar meanBB_BR, stdBB_BR;
    cv::meanStdDev(BB_YCbCr, meanBB_BR, stdBB_BR, overlapBB_BR);
    cv::Scalar meanBR_BB, stdBR_BB;
    cv::meanStdDev(BR_YCbCr, meanBR_BB, stdBR_BB, overlapBB_BR);

    // 4: overlapBR_FR (图像BR与FR) 
    cv::Scalar meanBR_FR, stdBR_FR;
    cv::meanStdDev(BR_YCbCr, meanBR_FR, stdBR_FR, overlapBR_FR);
    cv::Scalar meanFR_BR, stdFR_BR;
    cv::meanStdDev(FR_YCbCr, meanFR_BR, stdFR_BR, overlapBR_FR);

    // 5: overlapFR_FF (图像FR与FF)
    cv::Scalar meanFR_FF, stdFR_FF;
    cv::meanStdDev(FR_YCbCr, meanFR_FF, stdFR_FF, overlapFR_FF);
    cv::Scalar meanFF_FR, stdFF_FR;
    cv::meanStdDev(FF_YCbCr, meanFF_FR, stdFF_FR, overlapFR_FF);

    // 计算每个重叠区域两侧均值之差的绝对值
    cv::Scalar diff_FF_FL = scalarabs(meanFF_FL - meanFL_FF);
    cv::Scalar diff_FL_BL = scalarabs(meanFL_BL - meanBL_FL);
    cv::Scalar diff_BL_BB = scalarabs(meanBL_BB - meanBB_BL);
    cv::Scalar diff_BB_BR = scalarabs(meanBB_BR - meanBR_BB);
    cv::Scalar diff_BR_FR = scalarabs(meanBR_FR - meanFR_BR);
    cv::Scalar diff_FR_FF = scalarabs(meanFR_FF - meanFF_FR);

    // 这里根据需要，可以将各差值累加或进一步处理得到整体的光度差异指标
    // 示例：直接打印各重叠区域的均值差
    //std::cout << "Overlap FF-FL difference: " << diff_FF_FL << std::endl;
    //std::cout << "Overlap FL-BL difference: " << diff_FL_BL << std::endl;
    //std::cout << "Overlap BL-BB difference: " << diff_BL_BB << std::endl;
    //std::cout << "Overlap BB-BR difference: " << diff_BB_BR << std::endl;
    //std::cout << "Overlap BR-FR difference: " << diff_BR_FR << std::endl;
    //std::cout << "Overlap FR-FF difference: " << diff_FR_FF << std::endl;

    // 如果需要累加到全局曝光补偿指标中，可类似于如下操作：
    // exposure_diff += diff_FF_FL + diff_FL_BL + diff_BL_BB + diff_BB_BR + diff_BR_FR + diff_FR_FF;
}


void pc::Sticher::stich(cv::Mat& result, const std::vector<cv::Mat>& imgs, std::vector<cv::UMat>& remapImgs, int blend_type){
    // remap
    std::vector<cv::UMat> images_wraped_f(numCamera);
    std::vector<cv::UMat> resize_images_wraped_f(numCamera);

    for(int i=0;i<pc::numCamera;i++) {
        cv::remap(imgs[i], remapImgs[i], xMaps[i], yMaps[i], cv::INTER_LINEAR, cv::BORDER_REFLECT);
        // 恢复调试打印：检查映射矩阵
        double minVal, maxVal;
        cv::minMaxLoc(xMaps[i], &minVal, &maxVal);
       // std::cout << "Camera " << i << " xMap: min=" << minVal << " max=" << maxVal << std::endl;
        cv::minMaxLoc(yMaps[i], &minVal, &maxVal);
       // std::cout << "Camera " << i << " yMap: min=" << minVal << " max=" << maxVal << std::endl;
    }

    photometricAlignment_std_new(remapImgs[0], remapImgs[1], remapImgs[2], remapImgs[3], remapImgs[4], remapImgs[5]);
    // original exposure compensation
//    cv::Ptr<cv::detail::ExposureCompensator> compensator =
//            cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::CHANNELS);
//    compensator->feed(corners, remapImgs, maskMaps);    //得到曝光补偿器
//    for(int i=0;i<4;++i)    //应用曝光补偿器，对图像进行曝光补偿
//    {
//        compensator->apply(i, corners[i], remapImgs[i], maskMaps[i]);
//    }

    // resize
    for(int i=0;i<pc::numCamera;i++) {
//        remapImgs[i].convertTo(images_wraped_f[i], CV_32F);
        cv::resize(remapImgs[i], resize_images_wraped_f[i], pc::resizeStitchResultSize);
    }

// 查找最佳接缝
  //if(mySeamFinder->find_dp_temporal_fast(resize_images_wraped_f, resizeMaskMapsSeam, true))
    if(mySeamFinder->find_dp_temporal_fast(resize_images_wraped_f, resizeMaskMapsSeam, false))
    {
        for(int i=0;i<pc::numCamera;i++) {
            cv::resize(resizeMaskMapsSeam[i], maskMapsSeam[i], pc::stitchResultSize, 0, 0);
        }
    }

    switch(blend_type){
        case cv::detail::Blender::NO:
            blenderPtr = cv::detail::Blender::createDefault( cv::detail::Blender::NO);
            break;

        case cv::detail::Blender::FEATHER:
            blenderPtr = cv::makePtr<cv::detail::FeatherBlender>(0.5);
            break;

        case cv::detail::Blender::MULTI_BAND:
            blenderPtr = cv::makePtr<cv::detail::MultiBandBlender>( false, pc::numBands);
            break;

    }

    blenderPtr->prepare(cv::Rect(0,0,pc::stitchResultSize.width, pc::stitchResultSize.height));

    std::vector<cv::UMat> remapImgs_16(numCamera);

    for(int i=0;i<pc::numCamera;i++){
        remapImgs[i].convertTo(remapImgs_16[i], CV_16SC3);
        blenderPtr->feed(remapImgs_16[i], maskMapsSeam[i], cv::Point(0,0));
    }
    cv::Mat result_s, result_mask;
    blenderPtr->blend(result_s, result_mask);
    result_s.convertTo(result, CV_8U);
}


void pc::Sticher::stich_ori(cv::Mat& result, const std::vector<cv::Mat>& imgs, std::vector<cv::UMat>& remapImgs, int seam_type, int blend_type){
    // remap
    std::vector<cv::UMat> images_wraped_f(numCamera);
    std::vector<cv::UMat> resize_images_wraped_f(numCamera);
    omp_set_num_threads(4);
#pragma omp parallel for
    for(int i=0;i<pc::numCamera;i++) {
        cv::remap(imgs[i], remapImgs[i], xMaps[i], yMaps[i], cv::INTER_LINEAR, cv::BORDER_REFLECT);
//        cv::remap(imgs[i], remapImgs[i], xMaps[i], yMaps[i], cv::INTER_NEAREST, cv::BORDER_CONSTANT);
    }

    photometricAlignment_std(remapImgs[0], remapImgs[1], remapImgs[2], remapImgs[3]);

    // resize
    for(int i=0;i<pc::numCamera;i++) {
        remapImgs[i].convertTo(images_wraped_f[i], CV_32F);
        cv::resize(images_wraped_f[i], resize_images_wraped_f[i], pc::resizeStitchResultSize);
    }

    switch(seam_type){
        case cv::detail::SeamFinder::NO:
            for(int i=0;i<pc::numCamera;i++) {
                maskMaps[i].copyTo(maskMapsSeam[i]);
            }
            break;

        case  cv::detail::SeamFinder::VORONOI_SEAM:
            seamFinderPtr = cv::makePtr<cv::detail::VoronoiSeamFinder>();
            for(int i=0;i<pc::numCamera;i++) {
                maskMaps[i].copyTo(maskMapsSeam[i]);
                cv::resize(maskMapsSeam[i], resizeMaskMapsSeam[i], pc::resizeStitchResultSize);
            }
            seamFinderPtr->find(resize_images_wraped_f, corners, resizeMaskMapsSeam);
            for(int i=0;i<pc::numCamera;i++) {
                cv::resize(resizeMaskMapsSeam[i], maskMapsSeam[i], pc::stitchResultSize);
            }
            break;

        case  cv::detail::SeamFinder::DP_SEAM:
            seamFinderPtr = cv::makePtr<cv::detail::DpSeamFinder>();
            for(int i=0;i<pc::numCamera;i++) {
                maskMaps[i].copyTo(maskMapsSeam[i]);
                cv::resize(maskMapsSeam[i], resizeMaskMapsSeam[i], pc::resizeStitchResultSize);
            }
            seamFinderPtr->find(resize_images_wraped_f, corners, resizeMaskMapsSeam);
            for(int i=0;i<pc::numCamera;i++) {
                cv::resize(resizeMaskMapsSeam[i], maskMapsSeam[i], pc::stitchResultSize);
            }
            break;

        case 3:
            seamFinderPtr = cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
            for(int i=0;i<pc::numCamera;i++) {
                maskMaps[i].copyTo(maskMapsSeam[i]);
                cv::resize(maskMapsSeam[i], resizeMaskMapsSeam[i], pc::resizeStitchResultSize);
            }
//            seamFinderPtr->find(images_wraped_f, corners, maskMapsSeam);
            seamFinderPtr->find(resize_images_wraped_f, corners, resizeMaskMapsSeam);

            for(int i=0;i<pc::numCamera;i++) {
                cv::resize(resizeMaskMapsSeam[i], maskMapsSeam[i], pc::stitchResultSize);
            }

            break;
    }


    switch(blend_type){
        case cv::detail::Blender::NO:
            blenderPtr = cv::detail::Blender::createDefault( cv::detail::Blender::NO);
            break;

        case cv::detail::Blender::FEATHER:
            blenderPtr = cv::makePtr<cv::detail::FeatherBlender>(1);
            break;

        case cv::detail::Blender::MULTI_BAND:
            blenderPtr = cv::makePtr<cv::detail::MultiBandBlender>( false, pc::numBands);
            break;

    }

    blenderPtr->prepare(cv::Rect(0,0,pc::stitchResultSize.width, pc::stitchResultSize.height));

    std::vector<cv::UMat> remapImgs_16(numCamera);

    for(int i=0;i<pc::numCamera;i++){
        remapImgs[i].convertTo(remapImgs_16[i], CV_16SC3);
        blenderPtr->feed(remapImgs_16[i], maskMapsSeam[i], cv::Point(0,0));
    }
    cv::Mat result_s, result_mask;
    blenderPtr->blend(result_s, result_mask);
    result_s.convertTo(result, CV_8U);
}