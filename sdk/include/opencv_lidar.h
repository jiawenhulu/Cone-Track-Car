#pragma once
#include <iostream>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <highgui.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "string.h"
#include <stdlib.h>


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#define LEFTCONE 0
#define RIGHTCONE 1
#define ENDCONE 2
#define OTHER 3
#define CONEHEIGHT 110000

using namespace std;
using namespace cv;

static int RadarImageWidth = 640;
static int RadarImageHeight = 480;
static int MapWidth = 100;
static int MapHeight = 100;
//测量点数据结构，这个可以参考应用手册
struct scanDot {
    _u8   quality;
    float angle;
    float dist;
};

struct ConeInfo
{
    vector<int> x;  //x axis from image (from left to right)
    vector<int> y;  //y axis from image (from the top bottom)
    vector<int> u;  //u axis from lidar (forward)
    vector<int> v;  //v axis from lidar (from left to right)
    vector<int> width;
    vector<int> height;
    vector<vector<Point> >hulls_cone;
    vector<int> area;
    vector<int> distance;
    int goal;
    float angle;
    float speed;
    Mat img;
    int amount;
};


class LidarImage
{
public:
    vector<scanDot> scan_data; //保存每扫描一周的雷达数据
    float scan_speed;
    void scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency);
    ConeInfo draw(Mat camImage,vector<scanDot> scanData);
};

class ConeDetect
{
public:
    ConeInfo img_convert(Mat img_src);
    ConeInfo cone_convert(Mat img_src,int coneJudge);
};

class MiddleLane
{
public:
    ConeInfo path_lane(Mat img,ConeInfo left,ConeInfo right,ConeInfo obj);
};

ConeInfo bresenham(int x1, int y1, int x2, int y2);
Rect pointSetbboundingRect( const Mat& points );

extern Mat img_frame;
extern int endSignal;
extern vector<scanDot> scan_data;



