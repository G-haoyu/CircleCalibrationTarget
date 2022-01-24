#include "Calibrating.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <core/types_c.h>
#include <core/core_c.h>
#include <imgproc/imgproc_c.h>
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

Calibration::Calibration(){}

Calibration::Calibration(cv::Mat originImg, cv::Mat grayImg, cv::Mat binaryImg)
{
    this->originImg = originImg;
    this->grayImg = grayImg;
    this->binaryImg = binaryImg;
}

cv::Mat Calibration::getOriginImg(){ return this->originImg; }
cv::Mat Calibration::getGrayImg(){ return this->grayImg; }
cv::Mat Calibration::getBinaryImg(){ return this->binaryImg; }
std::vector<cv::Point2f> Calibration::getCorners(){ return this->corners; }
std::vector<cv::Point2f> Calibration::getInnerDots(){ return this->innerDots; }
void Calibration::setCorners(std::vector<cv::Point2f> corners){ this->corners = corners; }
void Calibration::setInnerDots(std::vector<cv::Point2f> innerDots){ this->innerDots = innerDots; }

CameraParams::CameraParams()
{
    this->cameraMatrix = Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    this->distCoeffs = Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
}

cv::Mat CameraParams::getCameraMatrix(){ return this->cameraMatrix; }
cv::Mat CameraParams::getDistCoeffs(){ return this->distCoeffs; }
std::vector<cv::Mat> CameraParams::getTvecsMat(){ return this->tvecsMat; }
std::vector<cv::Mat> CameraParams::getRvecsMat(){ return this->rvecsMat; }
void CameraParams::setCameraMatrix(cv::Mat cameraMatrix){ this->cameraMatrix = cameraMatrix; }
void CameraParams::setDistCoeffs(cv::Mat distCoeffs){ this->distCoeffs = distCoeffs; }
void CameraParams::setTvecsMat(std::vector<cv::Mat> tvecsMat){ this->tvecsMat = tvecsMat; }
void CameraParams::setRvecsMat(std::vector<cv::Mat> rvecsMat){ this->rvecsMat = rvecsMat; }


// 对数组排序（升序）
void sortArray(double a[], int length)
{
    int i, j, temp;
    for(i = 0; i < length; ++i)
    {
        for(j = i + 1; j < length; ++j)
        {
            if(a[j] < a[i])
            {
                temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
        }
    }
}

// 获得两点间的欧氏距离
double getDistance (cv::Point2f point1, cv::Point2f point2)
{
    double distance = sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
    return distance;
}

// 将point类型点转为point2f类型点
std::vector<cv::Point2f> point2point2f(std::vector<cv::Point> points)
{
    vector<Point2f> resPoints;
    for(int i = 0; i < points.size(); i++)
    {
        Point2f p = Point2f(points[i].x, points[i].y);
        resPoints.push_back(p);
    }
    return resPoints;
}

// 将point2f类型点转为point类型点
std::vector<cv::Point> point2f2point(std::vector<cv::Point2f> points)
{
    vector<Point> resPoints;
    for(int i = 0; i < points.size(); i++)
    {
        Point p = Point(points[i].x, points[i].y);
        resPoints.push_back(p);
    }
    return resPoints;
}

// 绘点或线
cv::Mat drawPoints(std::string title, cv::Mat img, std::vector<cv::Point2f> points, int pointSize, cv::Scalar scalar, bool isLine)
{
    Mat showImg;
    if(img.type() == 0)
        cvtColor(img, showImg, CV_GRAY2BGR);
    else
        showImg = img.clone();

    if(!isLine)
    {
        for(int i = 0; i < points.size(); i++)
        {
            circle(showImg, points[i], pointSize, scalar, -1);
        }  
    }
    else
    {
        vector<vector<Point>> contours;
        contours.push_back(point2f2point(points));
        drawContours(showImg, contours, -1, scalar, pointSize);
    }
    
    imwrite(title, showImg);
    return showImg;
}

// 获得当前图片能够正常标定的标定板面积下限
int getAreaThreshold(cv::Mat img)
{
    int minLine = img.rows > img.cols ? img.cols : img.rows;
    minLine = minLine / 6;
    return minLine * minLine;
}

// 对图像进行均值滤波
cv::Mat imgFilter_mean(cv::Mat imgSrc, int winSize)
{
    Mat imgRes;
    blur(imgSrc, imgRes, Size(winSize, winSize), Point(-1,-1), BORDER_DEFAULT);
    return imgRes;
}

// 获取二值图像，使用OTSU自适应阈值算法
cv::Mat getOtsu(cv::Mat sourceImg, int splitNum)
{
    Mat grayImg = sourceImg.clone();

    // 分割图像
    vector<Mat> subImgs;
    int srcHeight = grayImg.rows;
    int srcWidth = grayImg.cols;
    int subHeight = srcHeight / splitNum;
    int subWeight = srcWidth / splitNum;

    for(int i = 0; i < splitNum; i++)
    {
        for(int j = 0; j < splitNum; j++)
        {
            if(i*subWeight <= srcWidth && j*subHeight <= srcHeight)
            {
                Mat temImg = Mat::zeros(Size(subWeight, subHeight), CV_8U);
                Mat imgROI = grayImg(Rect(i*subWeight, j*subHeight, temImg.cols, temImg.rows));
                addWeighted(temImg, 1, imgROI, 1, 0, temImg);
                // OTSU
                threshold(temImg, temImg, 0, 255, THRESH_OTSU);
                temImg.copyTo(imgROI);
            }
            else
            {
                break;
            }
        }
    }
    return grayImg;
}

// 手动二值处理，严格二值化
cv::Mat strictBinary(cv::Mat img)
{
    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {
            if(int(img.at<uchar>(i, j)) > 127)
                img.at<uchar>(i, j) = 255;
            else
                img.at<uchar>(i, j) = 0;
        }
    }
    return img;
}

// 形态学开闭操作
cv::Mat noiceReduction(cv::Mat sourceImg, cv::MorphShapes shapes, cv::Size size, cv::MorphTypes operation)
{
    Mat kernel = getStructuringElement(shapes, size);
    Mat resImg;

    morphologyEx(sourceImg, resImg, operation, kernel);
    return resImg;
}

// 查找内五边形
std::vector<cv::Point2f> findPentagon(cv::Mat img)
{
    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point());

    // 查找几何图像
    int staticArea = getAreaThreshold(img);// 面积门限
    vector<vector<Point>> contours_poly;
    for(int i = 0; i < contours.size(); i++)
    {
        double acreage = contourArea(contours[i], true);
        if(acreage > staticArea)
        {
            vector<Point> contoursPoly;
            approxPolyDP(contours[i], contoursPoly, 8, true);
            contours_poly.push_back(contoursPoly);
        }
    }

    // 查找五边形
    vector<Point> contours_pentagon;// 五边形点，未排序
    for(int i = 0; i < contours_poly.size(); i++)
    {
        if(isPentagon(contours_poly[i]))
        {
            contours_pentagon = contours_poly[i];
            break;
        }
    }

    // 转point2f
    vector<Point2f> pentagonContours = point2point2f(contours_pentagon);// 五边形点，未排序

    return pentagonContours;
}

// 确定是否是五边形
bool isPentagon(std::vector<cv::Point> contours)
{
    bool isPentagon = false;
    if(contours.size() == 5)
    {
        Point p1 = Point(contours[0].x, contours[0].y);
        Point p2 = Point(contours[1].x, contours[1].y);
        Point p3 = Point(contours[2].x, contours[2].y);
        Point p4 = Point(contours[3].x, contours[3].y);
        Point p5 = Point(contours[4].x, contours[4].y);
        double lines[5];
        lines[0] = getDistance(p1, p2);
        lines[1] = getDistance(p2, p3);
        lines[2] = getDistance(p3, p4);
        lines[3] = getDistance(p4, p5);
        lines[4] = getDistance(p5, p1);
        sortArray(lines, 5);
        double shortRate = lines[0] / lines[1];
        double secondRate = lines[1] / lines[2];
        double thirdRate = lines[2] / lines[3];
        double longRate = lines[3] / lines[4];
        if(shortRate < 0.3 && secondRate > 0.7 && thirdRate > 0.8 && longRate > 0.8)
        {
            isPentagon = true;
        }
    }
    return isPentagon;
}

// 获得内四边形四点
std::vector<cv::Point2f> getInnerQuar(std::vector<cv::Point2f> contours)
{
    Point2f points[5];
    points[0] = Point2f(contours[0].x, contours[0].y);
    points[1] = Point2f(contours[1].x, contours[1].y);
    points[2] = Point2f(contours[2].x, contours[2].y);
    points[3] = Point2f(contours[3].x, contours[3].y);
    points[4] = Point2f(contours[4].x, contours[4].y);

    int m,n;// 短边两点下标
    double minDistance = getDistance(points[0], points[1]);
    m=0, n=1;
    for(int i = 1; i < 5; i++)
    {
        if(i == 4)
        {
            double distance = getDistance(points[i], points[0]);
            if(distance < minDistance)
            {
                m = i;
                n = 0;
                minDistance = distance;
            }
        }
        else
        {
            double distance = getDistance(points[i], points[i+1]);
            if(distance < minDistance)
            {
                m = i;
                n = i+1;
                minDistance = distance;
            }
        }
    }
    int w,u;// 两条线段的另两点
    w = (m - 1 + 5) % 5;
    u = (n + 1 + 5) % 5;

    Point2f crossPoint1 = crossPoint(points[m], points[w], points[n], points[u]);
    vector<Point2f> resContours;
    resContours.push_back(crossPoint1);
    resContours.push_back(points[u]);
    resContours.push_back(points[(u+1+5)% 5]);
    resContours.push_back(points[w]);
    return resContours;// 四边形点，已排序
}

// 四点获得两直线交点
cv::Point2f crossPoint(cv::Point2f point1, cv::Point2f point2, cv::Point2f point3, cv::Point2f point4)
{
	//计算点1，2形成直线与点3，4形成直线交点
	//如果平行或有无穷个交点就取点2和3的中间点
	int x, y;
	int X1 = point1.x - point2.x, Y1 = point1.y - point2.y, X2 = point3.x - point4.x, Y2 = point3.y - point4.y;
	if (X1*Y2 == X2*Y1)return Point((point2.x+point3.x)/2,(point2.y+point3.y)/2);
 
	int A = X1*point1.y - Y1*point1.x,B= X2*point3.y - Y2*point3.x;

	y = (A*Y2 - B*Y1) / (X1*Y2 - X2*Y1);
	x = (B*X1-A*X2) / (Y1*X2 - Y2*X1);
	return Point2f(x, y);
}

// 通过原图内四边形四点获得透视变换后正方形四点
std::vector<cv::Point2f> createRightRecPoints(std::vector<cv::Point2f> contours)
{
    Point2f p1 = Point(contours[0].x, contours[0].y);
    Point2f p2 = Point(contours[1].x, contours[1].y);
    Point2f p3 = Point(contours[2].x, contours[2].y);
    Point2f p4 = Point(contours[3].x, contours[3].y);
    double lines[4];
    lines[0] = getDistance(p1, p2);
    lines[1] = getDistance(p2, p3);
    lines[2] = getDistance(p3, p4);
    lines[3] = getDistance(p4, p1);

    sortArray(lines, 4);
    int shortLine = (int) lines[0];

    vector<Point2f> resContours;
    resContours.push_back(Point2f(0,0));
    resContours.push_back(Point2f(shortLine,0));
    resContours.push_back(Point2f(shortLine,shortLine));
    resContours.push_back(Point2f(0,shortLine));

    return resContours;
}

// 根据透视变换后的内四边形获得所有理想中心点
std::vector<cv::Point2f> getAllIdeaInnerDots(std::vector<cv::Point2f> contours)
{
    Point2f p1 = Point2f(contours[0].x, contours[0].y);
    Point2f p2 = Point2f(contours[1].x, contours[1].y);
    double shortLine = getDistance(p1, p2);
    double distance1 = shortLine*(2.0/19.0);
    double distance2 = shortLine*(2.5/19.0);

    vector<Point2f> points;

    Point2f a = Point2f(p1.x+distance1, p1.y+distance1);
    for(int j = 0; j < 7; j++)
    {
        points.push_back(a);
        for(int i = 0; i < 6; i++) 
        {
            points.push_back(Point(a.x+distance2, a.y));
            a = Point(a.x+distance2, a.y);
        }
        a = Point(p1.x+distance1, a.y+distance2);
    }

    return points;
}

// 获取所有圆点中心
std::vector<cv::Point2f> getAllCorePoints(cv::Mat img, cv::Mat binaryImg, std::vector<cv::Point2f> ideaInnerDots)
{
    double roiSize = 0.9;// 选取矩阵的大小

    Mat grayImg = img.clone();
    bitwise_not(binaryImg, binaryImg);
    // 准备模板
    for(int i = 0; i < binaryImg.rows; i++) {
        for(int j = 0; j < binaryImg.cols; j++) {
            if(int(binaryImg.at<uchar>( i, j))>125)
                binaryImg.at<uchar>( i, j) = 255;
            else
                binaryImg.at<uchar>( i, j) = 1;
        }
    }

    grayImg = grayImg.mul(binaryImg);// 模板乘


    vector<Point2f> rightPoints;

    double rightInterval = getDistance(ideaInnerDots[0], ideaInnerDots[1]);
    rightInterval = (int) (rightInterval * roiSize);

    for(int i = 0; i < ideaInnerDots.size(); i++)
    {
       rightPoints.push_back(getCore(grayImg, ideaInnerDots[i], rightInterval, rightInterval));
    }
    return rightPoints;
}

// 局部中心坐标转全局中心坐标
cv::Point2f getCore(cv::Mat img, cv::Point2f ideaPoint, int width, int height)
{
    Mat grayImg = img.clone();

    Mat targetImg = Mat::zeros(Size(width, height), CV_8U);
    Mat imgROI = grayImg(Rect(ideaPoint.x - width/2, ideaPoint.y - height/2, targetImg.rows, targetImg.cols));
    addWeighted(targetImg, 1, imgROI, 1, 0, targetImg);

    bitwise_not(targetImg, targetImg);
    Point2f center = grayCenter(targetImg);
    // stringstream ss;
    // ss<<ideaPoint.y<<ideaPoint.x;
    // imshow(ss.str(), targetImg);
    center.x = cvRound(ideaPoint.x - width/2 + center.x);
    center.y = cvRound(ideaPoint.y - height/2 + center.y);
    return center;
}

// 灰度重心法
cv::Point2f grayCenter(cv::Mat& img_gray)
{
	Point2f Center;
	int i, j;
	double sumval = 0; 
	MatIterator_<uchar> it, end;
	for (it = img_gray.begin<uchar>(), end = img_gray.end<uchar>(); it != end; it++)
	{
		sumval += (*it);
	}
	Center.x = Center.y = 0;
	double x = 0, y = 0;
	for (int i = 0; i < img_gray.cols; i++)
	{
		for (int j = 0; j < img_gray.rows; j++)
		{
			double s = img_gray.at<uchar>(j, i);
			x += i * s / sumval;
			y += j * s / sumval;
		}
	}
	Center.x = cvRound(x);
	Center.y = cvRound(y);
	return Center;
}

// 获取相机参数
CameraParams getCameraParams(std::vector<cv::Point2f> innerDots0, std::vector<cv::Point2f> ideaInnerDots, cv::Size imgSize)
{
    CameraParams cameraParams = CameraParams();
    vector<vector<Point2f>> innerDots;
    innerDots.push_back(innerDots0);
    vector<vector<Point3f>> objectPoints;
    objectPoints.push_back(getWorldPoints(ideaInnerDots));

    Mat cameraMatrix = Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    Mat distCoeffs = Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); 
    vector<Mat> tvecsMat;
    vector<Mat> rvecsMat;
    calibrateCamera(objectPoints, innerDots, imgSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, CALIB_FIX_K3);
    cameraParams.setCameraMatrix(cameraMatrix);
    cameraParams.setDistCoeffs(distCoeffs);
    cameraParams.setTvecsMat(tvecsMat);
    cameraParams.setRvecsMat(rvecsMat);

    return cameraParams;
}

// 获取世界坐标
std::vector<cv::Point3f> getWorldPoints(std::vector<cv::Point2f> contours_realPoints)
{
    vector<Point3f> realPoint;
    for(int i = 0; i < contours_realPoints.size(); i++)
    {
        Point3f tempPoint;
        tempPoint.x = contours_realPoints[i].x;
        tempPoint.y = contours_realPoints[i].y;
        tempPoint.z = 0;
        realPoint.push_back(tempPoint);
    }
    return realPoint;
}
