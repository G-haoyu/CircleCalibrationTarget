# 圆形标定板标定的opencv c++实现

## 实现环境

- 平台：win10

- VSCode V1.63.2

- cmake V3.18.4

- opencv V4.5.2

- mingw64 V8.1.0

## 文件说明

- Calibrating.h
  - Calibration类、所有使用的方法的声明
- Calibrating.cpp
  - Calbrating.h的实现

- AutoCalibrating.cpp
  - 主程序

## 实现步骤

### 图像预处理模块

目的是获得可标定的二值图像

#### 读取待标定图像

使用imread函数读取需要标定的图像

![bigWork_1](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_1.6ir6jfg7h7w0.webp)

#### 创建灰度图像

使用cvtColor函数将彩色图像转为灰度图像*COLOR_BGR2GRAY*

![bigWork_2](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_2.3au2vzbflo80.webp)

#### 进行均值模糊

使用blur函数对图像进行均值滤波处理，目的是减弱原图像中噪声对接下来的标定产生的影响

*不预先进行均值模糊可能会导致之后找不到标定板轮廓*

![bigWork_3](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_3.3nm3qv0oy5m0.webp)

#### OTSU自适应阈值二值化

使用OTSU自适应阈值算法获得获取二值图像，原理就是切割原图像为多个子图，对每个子图进行OTSU

这里使用自适应OTSU算法而不是标准二值化或是全局OTSU是考虑到图像拍摄时可能存在光照等影响，会导致寻找全局性质的阈值进行二值化会效果不佳，如使标定板图像残缺等，于是综合考虑实现难度和效果，选择使用自适应阈值的OTSU算法进行图像二值化

![bigWork_4](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_4.4vpcdkosacc0.webp)

#### 开操作和闭操作

对图像依次进行一次开操作和闭操作，将内部空点填补并连接边缘断点处

这一步并不是在整个标定过程中起到决定性作用，但增加对图像的开闭操作后能够对图片中的一些噪点和边缘残缺的现象进行抑制和修补，使后面要进行的边缘提取算法能够更加高效和准确（因为消除了小的独立噪点并连接了断点）

![bigWork_6](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_6.130wf5wwc5y8.webp)

#### 将二值图像反色，创建Calibration对象

使用bitwise_not函数进行反色处理，并创建未透视变换的Calibration对象

这里的Calibration是自定义的一个类，为了更加方便地管理寻找到的标定板边缘和内圆点的坐标，不用多次将这些坐标写入内存，造成内存的浪费

![bigWork_7](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_7.5xjvzr2ykd00.webp)

### 识别顶点模块

基于上一个模块得到的二值图像，在图像内寻找标定板位置

#### 查找内五边形顶点

先通过findContours查找轮廓，然后获得当前图片能够正常标定的标定板面积下限，再由面积下限作为阈值来进行多边形拟合，确定图像内是否存在大的多边形

如果存在则查找这些多边形是否是标定板的内五边形，这里通过边长关系进行限制。若存在则对五边形的各个顶点进行排序和标记，若不存在则返回相应提示并退出程序

![bigWork_8](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_8.492geljw5ye0.webp)

#### 确定内四边形四点

通过计算与五边形最短边两顶点的欧式距离之和的最小值获得四边形的四个顶点，并在图像上画出四边形的轮廓

![bigWork_9](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_9.wicxozaigc0.webp)

#### 透视变换

对图像进行透视变换，将四边形变换为正四边形，置于左上角，并通过原图内四边形四点获得透视变换后正方形四点

这一步的目的是方便后一步获得理想的圆心，因为标定板的各长度比值是固定的，所以很容易求出透视变换后的理想圆心得位置

![bigWork_10](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_10.2yip32wbqxy0.webp)

### 获取圆心模块

获得标定板内圆形的圆心坐标

#### 获得所有理想圆心

创建透视变换后的Calibration对象，根据透视变换后的内四边形获得所有理想中心点，并标记在图像中

理想圆心和实际的圆心还是存在较大差距的，当然理想圆心仍然有用，以理想圆心为中心圈出一个矩形框，作为每一个需要标定的圆心的感兴趣区域

![bigWork_12](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_12.4cb8fac41fm0.webp)

#### 灰度重心法确定圆心

截取矩形区域，与二值图相与，使用灰度重心法确定实际圆心，标记在图中

这一步灰度重心法之前我们进行了改良，将一开始得到的二值图像与我们的灰度图像进行相与的操作，这样我们就只保留了图像圆中的灰度信息，消除了圆外的灰度变化对接下来的灰度重心算法造成的影响

同时这里采用灰度重心法也是因为考虑到标定板图像拍摄时可能由于光线问题和具体畸变问题导致拍摄后的图像存在阴影和形状畸变的影响，使用灰度重心法能够较好的规避上述两种情况带来的影响

![bigWork_13](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_13.39kos2jy9w80.webp)

#### 反透视变换

对确定的中点进行反透视变换，得到完成圆点标记后的图像

![bigWork_14](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_14.5e4w4zau1rw0.webp)

#### 校准图像

获取相机参数，结合标定板数据，校准原始图像的畸变

![bigWork_15](https://cdn.jsdelivr.net/gh/G-haoyu/image-hosting@master/bigWork_15.6uokzsvin8o0.webp)

## 实现缺陷

直接使用两条延长线交点确定第四个点的方法存在误差，因为图像存在畸变

使用透视变换后再进行灰度重心法标定圆心存在一定误差，因为图像存在畸变

