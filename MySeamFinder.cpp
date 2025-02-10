#include "MySeamFinder.h"

/**
 * @brief pc::MySeamFinder::MySeamFinder
 * @param maskMaps
 * 构造函数，初始化MySeamFinder类的成员变量
 */
pc::MySeamFinder::MySeamFinder(std::vector<cv::UMat> &maskMaps){
    for(int i=0; i<pc::numCamera; i++){
        cv::Mat temp;
        maskMaps[i].getMat(cv::ACCESS_FAST).copyTo(temp);
        maskMaps_.push_back(temp);
    }

    for(int i=0; i<pc::numCamera; i++){
        cv::Mat temp;
        cv::bitwise_and(maskMaps[i], maskMaps[(i+1)%pc::numCamera], temp, maskMaps[i]);
        overlapMaskMaps_.push_back(temp);

        std::vector<std::vector<int> > temp_vv;
        std::vector<int> temp_seampos;
        std::vector<int> temp_seamIntensity;
        for(int r=0; r<temp.rows; r++){
            temp_seampos.push_back(0);
            temp_seamIntensity.push_back(0);
            std::vector<int> temp_v;
            for(int c=0; c<temp.cols; c++){
                if(temp.at<uchar>(r, c-1) > 0 && temp.at<uchar>(r, c) > 0 ){
                    temp_v.push_back(c);
                }
            }
            temp_vv.push_back(temp_v);
        }
        overlapRowCols_.push_back(temp_vv);
        lastframe_seamPos_.push_back(temp_seampos);
        lastframe_seamItensity_.push_back(temp_seamIntensity);
        lastframe_seamPosMaps.push_back(cv::Mat::zeros(cv::Size(temp.cols, temp.rows), CV_8U));
    }

    for(int i=0; i<4; i++){
        cv::Mat temp(pc::resizeStitchResultSize, CV_8UC1);
        human_saliency.push_back(temp);
    }

    // initialize_socket(); // Commented out
}

/* // Commented out
void pc::MySeamFinder::initialize_socket() {
    cli_sockfd=socket(AF_INET,SOCK_STREAM,0);
    if(cli_sockfd<0){
        fprintf(stderr,"socker Error:%s\n",strerror(errno));
        exit(1);
    }
    int addrlen=sizeof(struct sockaddr_in);
    char seraddr[14]="127.0.0.1";
    struct sockaddr_in ser_addr, cli_addr;
    bzero(&ser_addr,addrlen);
    ser_addr.sin_family=AF_INET;
    ser_addr.sin_addr.s_addr=inet_addr(seraddr);
    ser_addr.sin_port=htons(2222);
    if(connect(cli_sockfd,(struct sockaddr*)&ser_addr, addrlen)!=0)
    {
        fprintf(stderr,"Connect Error:%s\n",strerror(errno));
        close(cli_sockfd);
        exit(1);
    }
}

void pc::MySeamFinder::human_segmentation_socket(std::vector<cv::UMat> &remapImgs) {
    for(int i=0; i<4; i++){
        cv::Mat temp_mat = remapImgs[i].getMat(cv::ACCESS_FAST);
        unsigned  char* uch = temp_mat.data;
        send(cli_sockfd, uch, total_size,0);
        recv(cli_sockfd, recvb, 1024,0);
        unsigned char* ptr = recvb+1024;
        int recived_bytes = 1024;
        while(recived_bytes<total_size/3){
            recv(cli_sockfd, ptr, 1024,0);
            ptr +=1024;
            recived_bytes += 1024;
        }
        ptr = recvb;
        std::memcpy((void*) human_saliency[i].data, recvb, total_size/3*sizeof(unsigned char));
    }
}
*/ // Commented out

/**
 * @brief pc::MySeamFinder::translateTransform_x
 * @param src
 * @param dst
 * @param dx
 *  向右平移x
 */
void pc::MySeamFinder::translateTransform_x(cv::Mat const& src, cv::Mat& dst, int dx)   //向右平移x
{
    int rows = src.rows;
    int cols = src.cols;
    dst.create(rows, cols, src.type());
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            //平移后坐标映射到原图像
            int x = j - dx;
            //保证映射后的坐标在原图像范围内
            if (x >= 0&& x < cols)
                dst.at<uchar>(i,j) = src.at<uchar>(i,x);
        }
    }
}

/**
 * @brief pc::MySeamFinder::translateTransform_y
 * @param src
 * @param dst
 * @param dy
 * 向下平移y
 */
void pc::MySeamFinder::translateTransform_y(cv::Mat const& src, cv::Mat& dst, int dy)   //向下平移y
{
    int rows = src.rows;
    int cols = src.cols;
    dst.create(rows, cols, src.type());
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            //平移后坐标映射到原图像
            int y = i - dy;
            //保证映射后的坐标在原图像范围内
            if (y >= 0&& y < rows)
                dst.at<uchar>(i,j) = src.at<uchar>(y,j);
        }
    }
}

/**
 * @brief pc::MySeamFinder::find_dp_temporal_fast
 * @param remapImgs
 * @param maskMapsSeam
 * @param human
 * @return
 * 查找最佳接缝
 */
bool pc::MySeamFinder::find_dp_temporal_fast(std::vector<cv::UMat> &remapImgs, std::vector