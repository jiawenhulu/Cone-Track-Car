#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include "opencv_lidar.h"

#define BARRIERLEN 150

const int GOAL = 1;
const int CALPOINT = 12;
const float SPEED = 0.6;
const int n=100; // horizontal size of the map
const int m=100; // vertical size size of the map
const int GAP = 4;
int LeftEdge_u = int(n / 2 - 13);
int LeftEdge_v = int(m - 13);
int RightEdge_u = int(n / 2 + 14);
int RightEdge_v = int(m - 13);
int Central_u = int(n / 2);
int Central_v = int(m - 1);
int GlobalCenter[2] = {n / 2,m - 20};

ConeInfo Barrier_judge(ConeInfo obj);
ConeInfo middle_lane(ConeInfo left,ConeInfo right);


ConeInfo MiddleLane::path_lane(Mat img,ConeInfo left,ConeInfo right,ConeInfo obj)
{
    int leftInterval = int(left.u.size() / GAP);
    int rightInterval = int(right.u.size() / GAP);
    int lidarU = 0;
    ConeInfo left_point,right_point;
    ConeInfo middleLane;
    int middlePoint[2];
    middleLane.goal = int(n / 2);
    /*no cone*/
    if(left.u.size() == 0 && right.u.size() == 0)
    {
	middleLane.speed = 0;
    /*	ConeInfo barrPos = Barrier_judge(obj);
    	if(barrPos.u.size() == 0){
    		middleLane.speed = 0;
    	}
    	else{
    		for(int i = 0; i < barrPos.u.size(); i++)
    		{
    			lidarU += barrPos.u[i];
    		}
    		middleLane.goal = int(lidarU / barrPos.u.size());
    		middleLane.speed = 0.15;*/
    		//ROS_INFO("i:%d",middleLane.goal);
    	} 	    	
    }
    else{
       if(left.u.size() == 0)  //no leftcone
        {
        	middlePoint[0] = int((right.u[right.u.size() - 1] + LeftEdge_u) / 2);
        	middlePoint[1] = int((right.v[right.u.size() - 1] + LeftEdge_v) / 2);
            middleLane = bresenham(Central_u,Central_v,middlePoint[0],middlePoint[1]);         
        }
        else if(right.u.size() == 0)  //no rightcone
        {
        	middlePoint[0] = int((left.u[left.u.size() - 1] + RightEdge_u) / 2);
        	middlePoint[1] = int((left.v[left.u.size() - 1] + RightEdge_v) / 2);
            middleLane = bresenham(Central_u,Central_v,middlePoint[0],middlePoint[1]);              
        }
         else{
		    for(int i = 1;i <= GAP; i++)
		    {
		        if(leftInterval * i < left.u.size())
		        {
		            left_point.u.push_back(left.u[leftInterval * i - 1]);
		            left_point.v.push_back(left.v[leftInterval * i - 1]);
		        }
		        else{
		            left_point.u.push_back(left.u[left.u.size() - 1]);
		            left_point.v.push_back(left.v[left.u.size() - 1]);
		        }
		        if(rightInterval * i < right.u.size())
		        {
		            right_point.u.push_back(right.u[rightInterval * i - 1]);
		            right_point.v.push_back(right.v[rightInterval * i - 1]);
		        }
		        else{
		            right_point.u.push_back(right.u[right.u.size() - 1]);
		            right_point.v.push_back(right.v[right.u.size() - 1]);
		        }
		    }
		    middleLane = middle_lane(left_point,right_point);    
        }
        
        for(int i = 0;i < middleLane.u.size(); i++)
        {
            circle(img, Point(middleLane.u[i], middleLane.v[i]), 1, Scalar(255, 255, 255), -1);
        }

        if(middleLane.u.size() > (CALPOINT))
        {
            middleLane.goal = middleLane.u[CALPOINT];
            circle(img, Point(middleLane.u[CALPOINT], middleLane.v[CALPOINT]), 1, Scalar(0, 255, 255), -1);
        }
        else{
            vector<int>::iterator k_u = middleLane.u.end() - 1;
            middleLane.goal = *k_u;
        }
        middleLane.speed = SPEED;
    }
    middleLane.angle = float((n/2 - middleLane.goal) * 0.035);
    //middleLane.angle = float(atan(float(goalBias / float(m - middle_v)))*57.3);
    if(middleLane.angle > 0.35) middleLane.angle = 0.35;
    else if(middleLane.angle < -0.35)
        middleLane.angle = -0.35;
    
    //ROS_INFO("i:%f",middleLane.angle);
    return middleLane;
}


ConeInfo middle_lane(ConeInfo leftPoint,ConeInfo rightPoint)
{
    static int Point[4];
    //ConeInfo Point;
    ConeInfo pointLane,tempLane;
    Point[0] = int(n / 2);
    Point[1] = int(m - 1);
    for(int i = 0;i < GAP;i++)
    {
        vector<int>::iterator l_u = leftPoint.u.begin() + i;
        vector<int>::iterator l_v = leftPoint.v.begin() + i;
        vector<int>::iterator r_u = rightPoint.u.begin() + i;
        vector<int>::iterator r_v = rightPoint.v.begin() + i;
        Point[2] = int((*l_u + *r_u)/2);
        Point[3] = int((*l_v + *r_v)/2);
        tempLane = bresenham(Point[0],Point[1],Point[2],Point[3]);
        pointLane.u.insert(pointLane.u.end(),tempLane.u.begin(),tempLane.u.end());
        pointLane.v.insert(pointLane.v.end(),tempLane.v.begin(),tempLane.v.end());
        Point[0] = Point[2];
        Point[1] = Point[3];
    }

    return pointLane;
}


ConeInfo Barrier_judge(ConeInfo obj)
{
    int barGap = 0;
    ConeInfo barr;
    barr.amount = 0;
    for(int BarIndex = 0;BarIndex < obj.u.size();BarIndex++)
    {
        if((obj.u[BarIndex] - obj.u[BarIndex+1]) > BARRIERLEN || (obj.v[BarIndex] - obj.v[BarIndex+1]) > BARRIERLEN)
        {
            if(obj.u[BarIndex] < 1000 && obj.u[BarIndex] > -1000 &&
               obj.v[BarIndex] < 0 && obj.v[BarIndex] > -1500)
            {
                barr.u.push_back(int(-obj.u[BarIndex] / 50 + (MapWidth / 2)));
                barr.v.push_back(int(MapHeight - (-obj.v[BarIndex] / 50)));
            }
        }
    }
    return barr;
}



