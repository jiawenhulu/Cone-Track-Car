#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv_lidar.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using std::vector;

int leftLowH = 0,leftHighH = 10,leftLowS = 90,leftLowV = 110;
int rightLowH = 11,rightHighH = 32,rightLowS = 55,rightLowV = 50;
int endLowH = 90,endHighH = 110,endLowS = 70,endHighS = 190;
int endLowV = 190,endHighV =255;


Mat element = getStructuringElement(MORPH_RECT, Size(5, 5) );
ConeInfo cone_judge(Mat image);
bool convex_hull_pointing_up(vector<Point> contour);

ConeInfo ConeDetect::img_convert(Mat img_src)
{
    Mat img_rgb;
    Mat img_HSV;
    Mat left,right,red,end,temp;
    Mat img_thresh_opened,img_thresh_blurred,img_edges;
    ConeInfo Contours;
    vector<vector<Point> > color_contours;
    vector<Vec4i> hierarchy;

    cvtColor(img_src, img_rgb, CV_BGR2RGB);
    cvtColor(img_rgb, img_HSV, CV_RGB2HSV);    
    
    inRange(img_HSV, Scalar(leftLowH, leftLowS, leftLowV), Scalar(leftHighH, 255, 255), left);
    inRange(img_HSV, Scalar(rightLowH, rightLowS, rightLowV), Scalar(rightHighH, 255, 255), right);
    inRange(img_HSV, Scalar(endLowH, endLowS, endLowV), Scalar(endHighH, endHighS, endHighV), end);
    addWeighted(left, 0.5, right, 0.5, 0.0, temp);
    addWeighted(temp, 0.5, end, 0.5, 0.0, temp);
    morphologyEx(temp, img_thresh_opened, 2, element);
    //medianBlur(img_thresh_opened, img_thresh_blurred, 5);
    Canny(img_thresh_opened, img_edges, 20, 80, 3);
    findContours(img_edges, color_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0)); 
    for( int id = 0; id < color_contours.size(); id++ )
    {
    	Rect rect;
		rect = pointSetbboundingRect(Mat(color_contours[id]));
    	if(rect.area() > 800)
    	{    		
			Contours.x.push_back(rect.x);
			Contours.y.push_back(rect.y);
			Contours.width.push_back(rect.width);
			Contours.height.push_back(rect.height);
    	}	    
    }     
    return Contours;
}

ConeInfo ConeDetect::cone_convert(Mat img_src,int coneJudge)
{
    Mat img_rgb;
    Mat img_HSV;
    Mat left,right,red,end;
    ConeInfo Contours;

    cvtColor(img_src, img_rgb, CV_BGR2RGB);
    cvtColor(img_rgb, img_HSV, CV_RGB2HSV);
    
    inRange(img_HSV, Scalar(leftLowH, leftLowS, leftLowV), Scalar(leftHighH, 255, 255), left);
    inRange(img_HSV, Scalar(rightLowH, rightLowS, rightLowV), Scalar(rightHighH, 255, 255), right);
    inRange(img_HSV, Scalar(endLowH, endLowS, endLowV), Scalar(endHighH, endHighS, endHighV), end);
    
    if(coneJudge == LEFTCONE)
    {
        Contours = cone_judge(left);
    }
    else if(coneJudge == RIGHTCONE)
    {
        Contours = cone_judge(right);
    }
    else if(coneJudge == ENDCONE)
    {
        Contours = cone_judge(end);
    }
    
    return Contours;
}

ConeInfo cone_judge(Mat image)
{
    Mat img_all_convex_hulls;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    ConeInfo temp_cone;
    temp_cone.amount = 0;
    Mat img_thresh_opened,img_thresh_blurred,img_edges;

    morphologyEx(image, img_thresh_opened, 2, element);
    medianBlur(img_thresh_opened, img_thresh_blurred, 5);
    Canny(img_thresh_blurred, img_edges, 20, 80, 3);
    findContours(img_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    vector<vector<Point> >approx_contours(contours.size());
    vector<vector<Point> >all_convex_hulls(contours.size());
    vector<vector<Point> >convex_hulls_3to10(contours.size());
    vector<vector<Point> >convex_hulls_cone(contours.size());

    for( int i = 0; i < contours.size(); i++ )
    {
         approxPolyDP( Mat(contours[i]), approx_contours[i], 3, true );
    }
    for( int i = 0; i < approx_contours.size(); i++ )
    {
         convexHull( Mat(approx_contours[i]), all_convex_hulls[i], false );
    }

    for( int i = 0; i< all_convex_hulls.size(); i++ )
    {
         if(all_convex_hulls[i].capacity() >= 3 && all_convex_hulls[i].capacity() <= 10)
         {
           convexHull( Mat(all_convex_hulls[i]), convex_hulls_3to10[i], false );
         }
    }

    //ConeClear();
   for( int i = 0; i < convex_hulls_3to10.size(); i++ )
    {
  /*       Mat temp = Mat(convex_hulls_3to10[i]);
		Mat conMat(contours[i].size(), 2, CV_32F);
		for(int j = 0; j < contours[j].size(); j ++)
		{
		   conMat.at<float>(j, 0) = convex_hulls_3to10[i][j].x;
		   conMat.at<float>(j, 1) = convex_hulls_3to10[i][j].y;
		}
	   boundingRect(conMat);*/

         if(convex_hull_pointing_up(convex_hulls_3to10[i]))
         {
            Rect rect;
            rect = pointSetbboundingRect(Mat(convex_hulls_3to10[i]));
            temp_cone.x.push_back(rect.x);
            temp_cone.width.push_back(rect.width);
            temp_cone.y.push_back(rect.y);
            temp_cone.height.push_back(rect.height);
            convexHull( Mat(convex_hulls_3to10[i]), convex_hulls_cone[temp_cone.amount], false );
            temp_cone.hulls_cone.push_back(convex_hulls_cone[temp_cone.amount]);
            temp_cone.amount++;
         }
    }
    
     return temp_cone;
}

bool convex_hull_pointing_up(vector<Point> contour)
{
    vector<int> points_above_center(10);
    vector<int> points_below_center(10);
    Rect boundRect;
    boundRect = pointSetbboundingRect(Mat(contour));
    int up_index = 0;
    int down_index = 0;
    int left_x = 0;
    int right_x = 0;
    float aspect_ratio = static_cast<float>((boundRect.width / 1.0) / boundRect.height);
    if(boundRect.area() < 800)
    {
        return false;
    }
    else if(aspect_ratio < 0.8)
    {
        float vertical_center = boundRect.y + cvRound(boundRect.height/2.0);

        for(int i = 0; i < contour.capacity(); i++)
        {
            if(contour[i].y < vertical_center)
            {
                points_above_center[up_index] = contour[i].x;
                up_index++;
            }
            else if(contour[i].y >= vertical_center)
            {
                points_below_center[down_index] = contour[i].x;
                down_index++;
            }
        }
        left_x = points_below_center[down_index-1];
        right_x = points_below_center[down_index-1];

        for(int i = 0; i < down_index; i++)
        {
            if(points_below_center[i] < left_x)
            {
                left_x = points_below_center[i];
            }
            if(points_below_center[i] > right_x)
            {
                right_x = points_below_center[i];
            }
        }
        int up_left_x = points_above_center[up_index-1];
        int up_right_x = points_above_center[up_index-1];

         for(int i = 0; i < up_index; i++)
        {
            if(points_above_center[i] < up_left_x)
            {
                up_left_x = points_above_center[i];
            }
            if(points_above_center[i] > up_right_x)
            {
                up_right_x = points_above_center[i];
            }
            if(points_above_center[i] < left_x || points_above_center[i] > right_x)
            {
                return false;
            }
        }
    }
    else{
        return false;
    }
    return true;
}

//from OpenCv
Rect pointSetbboundingRect( const Mat& points )
{
    int npoints = points.checkVector(2);
    int depth = points.depth();
    //CV_Assert(npoints >= 0 && (depth == CV_32F || depth == CV_32S));

    int  xmin = 0, ymin = 0, xmax = -1, ymax = -1, i;
    bool is_float = depth == CV_32F;

    if( npoints == 0 )
        return Rect();

    const Point* pts = points.ptr<Point>();
    Point pt = pts[0];

#if CV_SSE4_2
    if(cv::checkHardwareSupport(CV_CPU_SSE4_2))
    {
        if( !is_float )
        {
            __m128i minval, maxval;
            minval = maxval = _mm_loadl_epi64((const __m128i*)(&pt)); //min[0]=pt.x, min[1]=pt.y

            for( i = 1; i < npoints; i++ )
            {
                __m128i ptXY = _mm_loadl_epi64((const __m128i*)&pts[i]);
                minval = _mm_min_epi32(ptXY, minval);
                maxval = _mm_max_epi32(ptXY, maxval);
            }
            xmin = _mm_cvtsi128_si32(minval);
            ymin = _mm_cvtsi128_si32(_mm_srli_si128(minval, 4));
            xmax = _mm_cvtsi128_si32(maxval);
            ymax = _mm_cvtsi128_si32(_mm_srli_si128(maxval, 4));
        }
        else
        {
            __m128 minvalf, maxvalf, z = _mm_setzero_ps(), ptXY = _mm_setzero_ps();
            minvalf = maxvalf = _mm_loadl_pi(z, (const __m64*)(&pt));

            for( i = 1; i < npoints; i++ )
            {
                ptXY = _mm_loadl_pi(ptXY, (const __m64*)&pts[i]);

                minvalf = _mm_min_ps(minvalf, ptXY);
                maxvalf = _mm_max_ps(maxvalf, ptXY);
            }

            float xyminf[2], xymaxf[2];
            _mm_storel_pi((__m64*)xyminf, minvalf);
            _mm_storel_pi((__m64*)xymaxf, maxvalf);
            xmin = cvFloor(xyminf[0]);
            ymin = cvFloor(xyminf[1]);
            xmax = cvFloor(xymaxf[0]);
            ymax = cvFloor(xymaxf[1]);
        }
    }
    else
#endif
    {
        if( !is_float )
        {
            xmin = xmax = pt.x;
            ymin = ymax = pt.y;

            for( i = 1; i < npoints; i++ )
            {
                pt = pts[i];

                if( xmin > pt.x )
                    xmin = pt.x;

                if( xmax < pt.x )
                    xmax = pt.x;

                if( ymin > pt.y )
                    ymin = pt.y;

                if( ymax < pt.y )
                    ymax = pt.y;
            }
        }

    }

    return Rect(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1);
}






