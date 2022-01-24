#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#ifndef CALIBRATING_H_INCLUDED
#define CALIBRATING_H_INCLUDED

class Calibration
{
private:
    cv::Mat originImg;
    cv::Mat grayImg;
    cv::Mat binaryImg;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2f> innerDots;

public:
    Calibration();
    Calibration(cv::Mat originImg, cv::Mat grayImg, cv::Mat binaryImg);
    cv::Mat getOriginImg();
    cv::Mat getGrayImg();
    cv::Mat getBinaryImg();
    std::vector<cv::Point2f> getCorners();
    std::vector<cv::Point2f> getInnerDots();
    void setCorners(std::vector<cv::Point2f> corners);
    void setInnerDots(std::vector<cv::Point2f> innerDots);
};

class CameraParams
{
private:
    cv::Mat cameraMatrix;// 相机内参数矩阵
    cv::Mat distCoeffs;// 摄像机的5个畸变系数：k1,k2,p1,p2,k3
    std::vector<cv::Mat> tvecsMat;  // 每幅图像的旋转向量
    std::vector<cv::Mat> rvecsMat;  // 每幅图像的平移向量

public:
    CameraParams();
    cv::Mat getCameraMatrix();
    cv::Mat getDistCoeffs();
    std::vector<cv::Mat> getTvecsMat();
    std::vector<cv::Mat> getRvecsMat();
    void setCameraMatrix(cv::Mat cameraMatrix);
    void setDistCoeffs(cv::Mat distCoeffs);
    void setTvecsMat(std::vector<cv::Mat> tvecsMat);
    void setRvecsMat(std::vector<cv::Mat> rvecsMat);
};

void sortArray(double a[], int length);// 对数组排序（升序）

double getDistance (cv::Point2f point1, cv::Point2f point2);// 获得两点间的欧氏距离

std::vector<cv::Point2f> point2point2f(std::vector<cv::Point> points);// 将point类型点转为point2f类型点

std::vector<cv::Point> point2f2point(std::vector<cv::Point2f> points);// 将point2f类型点转为point类型点

cv::Mat drawPoints(std::string title, cv::Mat img, std::vector<cv::Point2f> points, int pointSize, cv::Scalar scalar, bool isLine);// 绘点

int getAreaThreshold(cv::Mat img);// 获得当前图片能够正常标定的标定板面积下限

cv::Mat imgFilter_mean(cv::Mat imgSrc, int winSize);// 对图像进行高斯滤波

cv::Mat getOtsu(cv::Mat sourceImg, int splitNum);// 获取二值图像，使用OTSU自适应阈值算法

cv::Mat strictBinary(cv::Mat img);// 手动二值处理，严格二值化

cv::Mat noiceReduction(cv::Mat sourceImg, cv::MorphShapes shapes, cv::Size size, cv::MorphTypes operation);// 形态学开闭操作

std::vector<cv::Point2f> findPentagon(cv::Mat img);// 查找内五边形

bool isPentagon(std::vector<cv::Point> contours);// 确定是否是五边形

std::vector<cv::Point2f> getInnerQuar(std::vector<cv::Point2f> contours);// 获得内四边形四点

cv::Point2f crossPoint(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3, cv::Point2f point4);// 四点获得两直线交点

std::vector<cv::Point2f> createRightRecPoints(std::vector<cv::Point2f> contours);// 通过原图内四边形四点获得透视变换后正方形四点

std::vector<cv::Point2f> getAllIdeaInnerDots(std::vector<cv::Point2f> contours);// 根据透视变换后的内四边形获得所有理想中心点

std::vector<cv::Point2f> getAllCorePoints(cv::Mat img, cv::Mat binaryImg, std::vector<cv::Point2f> ideaInnerDots);// 获取所有圆点中心

cv::Point2f getCore(cv::Mat img, cv::Point2f ideaPoint, int width, int height);// 局部中心坐标转全局中心坐标

cv::Point2f grayCenter(cv::Mat& img_gray);// 灰度重心法

CameraParams getCameraParams(std::vector<cv::Point2f> innerDots0, std::vector<cv::Point2f> ideaInnerDots, cv::Size imgSize);// 获取相机参数

std::vector<cv::Point3f> getWorldPoints(std::vector<cv::Point2f> contours_realPoints);// 获取世界坐标

#include "Calibrating.cpp"

#endif