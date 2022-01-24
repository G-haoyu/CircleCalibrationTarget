#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "Calibrating.h"

using namespace cv;
using namespace std;

int main( int argc, char* argv[] )
{

    /* 0、读取传入参数 */
    string path;
    if(argc == 3 && strcmp(argv[1], "-i") == 0){
        path = argv[2];
        cout<<argv[2];
    } 
    else
        exit(0);

    /* 1、读取待标定图像 */
    Mat img = imread(path);// "F:/opencv_c++_workspace/test9.jpg"
    imwrite("bigWork_1.jpg", img);

    /* 2、创建灰度图像 */
    Mat grayImg;
    cvtColor(img, grayImg, COLOR_BGR2GRAY);
    imwrite("bigWork_2.jpg", grayImg);

    /* 3、均值模糊 */
    Mat grayImg_mean = imgFilter_mean(grayImg, 4);
    imwrite("bigWork_3.jpg", grayImg_mean);

    /* 4、OTSU自适应阈值二值化 */
    Mat binaryImg = getOtsu(grayImg_mean, 1);
    binaryImg = strictBinary(binaryImg);
    imwrite("bigWork_4.jpg", binaryImg);

    /* 5、开操作
    作用：将内部空点填补
    */
    Mat binaryImg_denoise = noiceReduction(binaryImg, MORPH_RECT, Size(3,3), MORPH_OPEN);
    imwrite("bigWork_5.jpg", binaryImg_denoise);

    /* 6、闭操作
    作用：连接边缘断点处
    */
    binaryImg_denoise = noiceReduction(binaryImg, MORPH_RECT, Size(3,3), MORPH_CLOSE);
    imwrite("bigWork_6.jpg", binaryImg_denoise);

    /* 7、将二值图像反色，创建未透视变换的Calibration对象 */
    bitwise_not(binaryImg_denoise, binaryImg_denoise);
    Calibration calibration_normal = Calibration(img, grayImg, binaryImg_denoise);
    imwrite("bigWork_7.jpg", binaryImg_denoise);

    /* 8、查找内五边形顶点 */
    vector<Point2f> pentagonContours = findPentagon(calibration_normal.getBinaryImg());
    Mat showImg = drawPoints("bigWork_8.jpg", calibration_normal.getOriginImg(), pentagonContours, 4, Scalar(0,0,255), false);
    if(pentagonContours.size() < 1){
        cout<<"未找到圆形标定板的内五边形！"<<endl;
        exit(0);
    }


    /* 9、确定内四边形四点 */
    vector<Point2f> quadContours = getInnerQuar(pentagonContours);
    calibration_normal.setCorners(quadContours);
    showImg = drawPoints("bigWork_9.jpg", showImg, calibration_normal.getCorners(), 1, Scalar(0,0,255), true);

    /* 10、透视变换 */
    vector<Point2f> rightRecContours = createRightRecPoints(calibration_normal.getCorners());
    Mat warpMatrix = getPerspectiveTransform(calibration_normal.getCorners(), rightRecContours);// 变换矩阵
    Mat warpImg, warpGrayImg, warpBinaryImg;
    warpPerspective(calibration_normal.getOriginImg(), warpImg, warpMatrix, warpImg.size(), INTER_LINEAR, BORDER_CONSTANT);
    warpPerspective(calibration_normal.getGrayImg(), warpGrayImg, warpMatrix, warpGrayImg.size(), INTER_LINEAR, BORDER_CONSTANT);
    warpPerspective(calibration_normal.getBinaryImg(), warpBinaryImg, warpMatrix, warpBinaryImg.size(), INTER_LINEAR, BORDER_CONSTANT);
    
    Mat showImg2;
    warpPerspective(showImg, showImg2, warpMatrix, showImg2.size(), INTER_LINEAR, BORDER_CONSTANT);
    showImg = showImg2;
    imwrite("bigWork_10.jpg", showImg);


    /* 11、创建透视变换后的Calibration对象 */
    Calibration calibration_warp = Calibration(warpImg, warpGrayImg, warpBinaryImg);
    calibration_warp.setCorners(rightRecContours);

    /* 12、获得所有理想圆心 */
    vector<Point2f> ideaInnerDots = getAllIdeaInnerDots(calibration_warp.getCorners());
    showImg = drawPoints("bigWork_12.jpg", showImg, ideaInnerDots, 2, Scalar(0,0,255), false);

    /* 13、截取矩形区域，与二值图相与，灰度重心法确定圆心 */
    vector<Point2f> realInnerDots = getAllCorePoints(calibration_warp.getGrayImg(), calibration_warp.getBinaryImg(), ideaInnerDots);
    calibration_warp.setInnerDots(realInnerDots);
    showImg = drawPoints("bigWork_13.jpg", showImg, realInnerDots, 1, Scalar(0,255,0), false);

    /* 14、对确定的中点进行反透视变换 */
    Mat warpMatrix_reverse = getPerspectiveTransform(calibration_warp.getCorners(), calibration_normal.getCorners());// 反变换矩阵
    vector<Point2f> realInnerDots_noReverse;
    perspectiveTransform(calibration_warp.getInnerDots(), realInnerDots_noReverse, warpMatrix_reverse);
    perspectiveTransform(ideaInnerDots, ideaInnerDots, warpMatrix_reverse);
    calibration_normal.setInnerDots(realInnerDots_noReverse);
    showImg = drawPoints("bigWork_14.jpg", calibration_normal.getOriginImg(), calibration_normal.getInnerDots(), 1, Scalar(0,255,0), false);

    /* 15、获取相机参数，校准图像 */
    CameraParams cameraParams = getCameraParams(calibration_normal.getInnerDots(), ideaInnerDots, calibration_normal.getOriginImg().size());
    Mat finalImg;
    undistort(calibration_normal.getOriginImg(), finalImg, cameraParams.getCameraMatrix(), cameraParams.getDistCoeffs());
    imwrite("bigWork_15.jpg", finalImg);

    cout << "旋转向量:\n" << cameraParams.getTvecsMat()[0] << endl;
    cout << "平移向量:\n" << cameraParams.getRvecsMat()[0] << endl;
    cout << "相机内参数矩阵:\n" << cameraParams.getCameraMatrix() << endl;
    cout << "相机畸变系数:\n" << cameraParams.getDistCoeffs() << endl;

    waitKey();
    return 0;
}
