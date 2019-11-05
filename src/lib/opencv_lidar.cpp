#include "opencv_lidar.h"
#define PI 3.141592653
#define RECT_HEIGHT 160000  //140000
#define RECT_WIDTH 120000   //112000

Mat img_frame;
ConeInfo Last_Left, Last_Right, Last_End; 
//vector<scanDot> scan_data;
int endSignal = 0;
int Counting = 0;

float a[9] = {-7.2,-4.4,264.8,-0.021,-1.9,1454,-0.00043,-0.0135,1};

ConeInfo Cone_position(Mat img,ConeInfo contours_cone,ConeInfo lidar_point,int color);
ConeInfo Draw_lane(Mat img_src, ConeInfo obj, int obj_size, int color);
//ConeInfo sort_distance(ConeInfo value_buf, int obj_size);
ConeInfo sort_cone(ConeInfo value_buf, int obj_size);
ConeInfo Combin_lane(ConeInfo left,ConeInfo right);
//ConeInfo bresenham(int x1, int y1, int x2, int y2);
Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5) );


int abs (int n)
{
    return ( (n>0) ? n : ( n * (-1)));
}

template <typename T>
int medianFilter(vector<T>&value_buf, int size)
{
    int ave_value = 0,sum_value = 0;
     for (int out = 0; out < size-1; out++)
     {
      for (int in = 0; in < size-1-out; in++)
      {
         if ( value_buf[in]>value_buf[in+1] )
         {
            swap(value_buf[in],value_buf[in+1]);
         }
      }
    }
    for(int i = 1;i < size-1;i++)
    {
        sum_value += value_buf[i];
    }
    ave_value = sum_value/(size -2);
    return ave_value;
}

//turn orignal lidardata to distance and angle
void LidarImage::scanData(rplidar_response_measurement_node_t *buffer, size_t count, float frequency)
{
    scan_data.clear();
    for (int pos = 0; pos < (int)count; ++pos) {
        scanDot dot;
        if (!buffer[pos].distance_q2) continue;

        dot.quality = (buffer[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        dot.angle = (buffer[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
        dot.dist = buffer[pos].distance_q2 / 4.0f;
        scan_data.push_back(dot);
    }

    scan_speed = frequency;
}

ConeInfo LidarImage::draw(Mat camImage, vector<scanDot> scanData)
{
    Mat map_dst,camOrignal;
	ConeDetect coneDetect;
    int x = 0, y = 0, u = 0, v = 0;
    double theta = 0, rho = 0, lambda = 0;
	ConeInfo lidarPoint;
	ConeInfo left_cone,right_cone,end_cone;//Cone in map
	ConeInfo rectInfo,coneContours;
	ConeInfo pathLane;
	camOrignal = camImage.clone();

	int coneJudge = 3;

    //draw lidarpoint
    for (int i = 0; i < scanData.size(); i++)
    {
        scanDot dot;
        dot = scanData[i];

        if(dot.angle > 120 & dot.angle < 240)
       {
			theta = dot.angle*PI / 180;
			rho = dot.dist;
			u = (int)(rho*sin(theta));
			v = (int)(rho*cos(theta));

			lambda = u*a[6] + v*a[7] + a[8];
			x = int((u*a[0] + v*a[1] + a[2]) / lambda);
			y = int((u*a[3] + v*a[4] + a[5])  / lambda);

			lidarPoint.u.push_back(u);
			lidarPoint.v.push_back(v);
			lidarPoint.x.push_back(x);
			lidarPoint.y.push_back(y);
			lidarPoint.distance.push_back(rho);
			//ROS_INFO(":[%d,%d]", u, v);
			//circle(camImage, Point(x, y), 1, Scalar(255, 255, 0), -1, 8, 0);//draw lidarpoint
       }
    }

    coneContours = coneDetect.img_convert(camImage);

    left_cone = Cone_position(camImage,coneContours,lidarPoint,LEFTCONE);
    right_cone = Cone_position(camImage,coneContours,lidarPoint,RIGHTCONE);
    end_cone = Cone_position(camImage,coneContours,lidarPoint,ENDCONE);
    if(left_cone.u.size() >0 || right_cone.u.size() > 0)
    {
    	Last_Left.u.assign(left_cone.u.begin(),left_cone.u.end());
    	Last_Left.v.assign(left_cone.v.begin(),left_cone.v.end());
    	Last_Right.u.assign(right_cone.u.begin(),right_cone.u.end());
    	Last_Right.v.assign(right_cone.v.begin(),right_cone.v.end());
    }
    if(Counting > 15){           //clear the last info when it always without cone in 1s
		Last_Left.u.clear();    		
		Last_Left.v.clear();
		Last_Right.u.clear();
		Last_Right.v.clear();
	}

    ConeInfo left_lane,right_lane;
    
    MiddleLane middleLane;
    Mat map_img(MapWidth, MapHeight, CV_8UC3, Scalar(0, 0, 0));
    if(left_cone.u.size() == 0 && right_cone.u.size() == 0)
    {
    	left_lane = Draw_lane(map_img,Last_Left,Last_Left.u.size(),LEFTCONE);
    	right_lane = Draw_lane(map_img,Last_Right,Last_Right.u.size(),RIGHTCONE);
    	Counting++;
    }
    else{
    	left_lane = Draw_lane(map_img,left_cone,left_cone.u.size(),LEFTCONE);
    	right_lane = Draw_lane(map_img,right_cone,right_cone.u.size(),RIGHTCONE);
    }
    
    //two_lanes = Combin_lane(left_lane,right_lane);
    pathLane = middleLane.path_lane(map_img,left_lane,right_lane,lidarPoint);
    if(end_cone.v.size() ==2 && (end_cone.v[0] > 80 || end_cone.v[1] > 80))
    {
    	pathLane.speed = 0;
    	pathLane.angle = 0;
    	
    	if(endSignal == 0){
    	      	
    	  system("rosrun map_server map_saver -f map111");
    	}
    	endSignal = 1;
    }
    ROS_INFO("End signal:%d",endSignal);
    resize(map_img,map_dst,Size(map_img.cols*5,map_img.rows*5),0,0,INTER_LINEAR);
    //imshow("Map",map_dst);
    pathLane.img = map_dst;
    return pathLane;
}

ConeInfo Cone_position(Mat img,ConeInfo contours_cone,ConeInfo lidar_point,int color)
{
    vector<int> lidarConeU;
	vector<int> lidarConeV;
	ConeInfo conePosition,tempRoi;
	ConeDetect coneDetect;
	Mat ConeRoi,roi_RGB,roi_HSV;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> convex_hulls;
	int lidarpt = 0;
	int positionX = 0, positionY = 0;
	int u_ave = 0,v_ave = 0;
	int u_total = 0,v_total = 0;

	//ROS_INFO("coneinfo,%d", contours_cone.x.size());
    for(int cone_amount = 0; cone_amount < contours_cone.x.size(); cone_amount += 2)
    {
        int colStart = contours_cone.x[cone_amount] - 15;
        int rowStart = contours_cone.y[cone_amount] - 15;
        int colEnd = contours_cone.x[cone_amount] + contours_cone.width[cone_amount] +15;
        int rowEnd = contours_cone.y[cone_amount] + contours_cone.height[cone_amount] +15;
        if(rowStart < 0) rowStart = 0;
        if(colStart < 0) colStart = 0;
        if(rowEnd > RadarImageHeight) rowEnd = RadarImageHeight;
        if(colEnd > RadarImageWidth) colEnd = RadarImageWidth;
        
        ConeRoi = img(Range(rowStart,rowEnd),Range(colStart,colEnd));
        tempRoi = coneDetect.cone_convert(ConeRoi,color);
        //ROS_INFO("tempRoiinfo,%d", tempRoi.x.size());
        //for(int id = 0; id < tempRoi.x.size(); id += 2)
        if(tempRoi.x.size() > 0)
        {
            for(int ptId = 0; ptId < tempRoi.hulls_cone[0].size(); ptId++)
            {
            	Point roiHullPt,hullPt;
		        roiHullPt = tempRoi.hulls_cone[0][ptId];
		        hullPt.x = roiHullPt.x + colStart;
		        hullPt.y = roiHullPt.y + rowStart;
		        convex_hulls.push_back(hullPt);
            }
        	for (int i = 0; i < lidar_point.x.size(); i++)  
		    {		        
		        positionX = contours_cone.x[cone_amount];
		        positionY = contours_cone.y[cone_amount];
		        if(color == LEFTCONE)
		        {
		            rectangle(img,Point(positionX, positionY),Point(positionX + tempRoi.width[0],positionY + tempRoi.height[0]),Scalar(0, 0, 255),1,8,0);
		        }
                else if(color == RIGHTCONE)
		        {
		            rectangle(img,Point(positionX, positionY),Point(positionX + tempRoi.width[0],positionY + tempRoi.height[0]),Scalar(0, 255, 0),1,8,0);
		        }
		        else if(color == ENDCONE)
		        {
		            rectangle(img,Point(positionX, positionY),Point(positionX + tempRoi.width[0],positionY + tempRoi.height[0]),Scalar(255, 0, 0),1,8,0);
		        }
		        if(pointPolygonTest(convex_hulls,Point(lidar_point.x[i],lidar_point.y[i]),false) == 1) //if lidarpoint in cone's contours
		        {		        	
		            lidarConeU.push_back(lidar_point.u[i]);
		            lidarConeV.push_back(lidar_point.v[i]);
		            lidarpt++;
		        }
		        circle(img, Point(lidar_point.x[i], lidar_point.y[i]), 1, Scalar(255, 255, 0), -1, 8, 0);
		    }
		    
		    if(lidarpt != 0)
		    {
		        if(lidarpt > 2)
		        {
		            u_ave = -medianFilter(lidarConeU,lidarpt);
		            v_ave = -medianFilter(lidarConeV,lidarpt);
		        }
		        else{
		            for(int ld = 0; ld < lidarpt; ld++)
		            {
		                u_total += lidarConeU[ld];
		                v_total += lidarConeV[ld];
		            }
		            u_ave = int(-u_total/lidarpt);
		            v_ave = int(-v_total/lidarpt);
		        }	
		        	        
		        conePosition.u.push_back(int(u_ave / 50 + (MapWidth / 2)));
		        conePosition.v.push_back(int(MapHeight - (v_ave / 50)));
		        Point pt_position = Point(contours_cone.x[cone_amount], contours_cone.y[cone_amount]-5);
		        char temp[16];
		        sprintf(temp, "(%d,%d)", u_ave, v_ave);
		        putText(img, temp, pt_position, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);
		    }
		    
		    u_total = 0;
		    v_total = 0;
		    lidarpt = 0;
		    lidarConeU.clear();
		    lidarConeV.clear();
		    convex_hulls.clear();
		    Counting = 0;
	    }    
	    //rectangle(img,Point(colStart, rowStart),Point(colEnd,rowEnd),Scalar(0, 0, 255),1,8,0);  
    }
    
    
    return conePosition;
}


ConeInfo Draw_lane(Mat img_src, ConeInfo obj_cone, int obj_size, int color)
{
    ConeInfo obj_temp;
    ConeInfo point_lane;
    int counter = obj_size;
    //obj_cone = sort_cone(obj,obj_size);  //find the cone from near to far
    int test = 0;


    if(obj_size >= 1)
    {
        obj_cone.u.insert(obj_cone.u.begin(),MapWidth / 2);
        obj_cone.v.insert(obj_cone.v.begin(),MapHeight);

        //cone_distance.push_back(0); //the first distance is 0(from first cone to first cone)
        for(int i = 0; i < obj_size-1; i++)
        {
            vector<int> cone_distance; //the distance between two cone
            for(int j = 1;j < counter+1;j++)
            {
                cone_distance.push_back((obj_cone.u[0] - obj_cone.u[j])*(obj_cone.u[0] - obj_cone.u[j])
                                  + (obj_cone.v[0] - obj_cone.v[j])*(obj_cone.v[0] - obj_cone.v[j]));
            }
            for(int out = 0; out < counter-1; out++)
            {
                for(int in = 0; in < counter-1-out; in++)
                {
                    if ( cone_distance[in] > cone_distance[in+1] )
                    {
                        swap(obj_cone.v[in+1],obj_cone.v[in+2]);
                        swap(obj_cone.u[in+1],obj_cone.u[in+2]);
                        swap(cone_distance[in],cone_distance[in+1]);
                    }
                }
            }

            obj_temp.u.push_back(obj_cone.u[1]);
            obj_temp.v.push_back(obj_cone.v[1]);
            vector<int>::iterator k_u = obj_cone.u.begin();
            obj_cone.u.erase(k_u);//delete the first element
            vector<int>::iterator k_v = obj_cone.v.begin();
            obj_cone.v.erase(k_v);//delete the first element

        }
        obj_temp.u.push_back(obj_cone.u[1]);
        obj_temp.v.push_back(obj_cone.v[1]);

        if(color == LEFTCONE)
        {
            obj_temp.u.insert(obj_temp.u.begin(),MapWidth / 2 - 8);
            obj_temp.v.insert(obj_temp.v.begin(),MapHeight - 1);
        }
        else if(color == RIGHTCONE)
        {
            obj_temp.u.insert(obj_temp.u.begin(),MapWidth / 2 + 8);
            obj_temp.v.insert(obj_temp.v.begin(),MapHeight - 1);
        }

        for(int i = 0;i < obj_temp.u.size() - 1; i++)
        {
            //point_lane.u.push_back(obj_temp.u[i]);
            //point_lane.v.push_back(obj_temp.v[i]);
            ConeInfo temp_Lane;
            if(obj_temp.u.size() > 1)
            {
                temp_Lane = bresenham(obj_temp.u[i],obj_temp.v[i],obj_temp.u[i +1],obj_temp.v[i +1]);
            }
            point_lane.u.insert(point_lane.u.end(),temp_Lane.u.begin(),temp_Lane.u.end());
            point_lane.v.insert(point_lane.v.end(),temp_Lane.v.begin(),temp_Lane.v.end());
        }
       // point_lane.u.push_back(obj_temp.u[obj_size - 1]);
       // point_lane.v.push_back(obj_temp.v[obj_size - 1]);
        if(color == LEFTCONE)
        {
            for(int i = 0; i < point_lane.u.size(); i++)
            {
                circle(img_src, Point(point_lane.u[i], point_lane.v[i]), 1, Scalar(0, 0, 255), -1);
            }
        }
        else if(color == RIGHTCONE)
        {
            for(int i = 0; i < point_lane.u.size(); i++)
            {
                circle(img_src, Point(point_lane.u[i], point_lane.v[i]), 1, Scalar(0, 255, 255), -1);
            }
        }
    }
    return point_lane;
}



ConeInfo Combin_lane(ConeInfo left,ConeInfo right)
{
    ConeInfo combine;
    combine.u.insert(combine.u.end(),left.u.begin(),left.u.end());
    combine.v.insert(combine.v.end(),left.v.begin(),left.v.end());
    combine.u.insert(combine.u.end(),right.u.begin(),right.u.end());
    combine.v.insert(combine.v.end(),right.v.begin(),right.v.end());
    return combine;
}
/*
ConeInfo sort_distance(ConeInfo value_buf, int obj_size)
{
     for (int out = 1; out < obj_size; out++)
     {
      for (int in = 0; in < obj_size - out; in++)
      {
         if ( value_buf.distance[in] > value_buf.distance[in+1] )
         {
            swap(value_buf.distance[in],value_buf.distance[in+1]);
            swap(value_buf.v[in],value_buf.v[in+1]);
            swap(value_buf.u[in],value_buf.u[in+1]);
         }
      }
     }

    return value_buf;
}*/

ConeInfo sort_cone(ConeInfo value_buf, int obj_size)
{
     for (int out = 1; out < obj_size; out++)
     {
      for (int in = 0; in < obj_size - out; in++)
      {
         if ( value_buf.v[in] < value_buf.v[in+1] )
         {
            swap(value_buf.v[in],value_buf.v[in+1]);
            swap(value_buf.u[in],value_buf.u[in+1]);
         }
      }
     }

    return value_buf;
}

ConeInfo bresenham(int x1, int y1, int x2, int y2)
{
    ConeInfo Line;
    // Find Delta
    float dx = x2-x1;
    float dy = y2-y1;
    // Find Signs
    int sx = (dx>=0) ? 1 : (-1);
    int sy = (dy>=0) ? 1 : (-1);
    // Get Initial Points
    float x = x1;
    float y = y1;
    // Flag to check if swapping happens
    int isSwaped = 0;
    // Swap if needed
    if(abs(dy) > abs(dx))
    {
        // swap dx and dy
        float tdx = dx;
        dx =dy;
        dy = tdx;

        isSwaped = 1;
    }
    // Decision parameter
    float p = 2*(abs(dy)) - abs(dx);
    //Print Initial Point
    Line.u.push_back(x1);
    Line.v.push_back(y1);
    // Loop for dx times
    for(int i = 0; i<= abs(dx);i++)
    {
        // Depending on decision parameter
        if(p < 0)
        {
            if(isSwaped == 0){
                x = x + sx;
                Line.u.push_back(x);
                Line.v.push_back(y);
            }
            else{
                y = y+sy;
                Line.u.push_back(x);
                Line.v.push_back(y);
            }
            p = p + 2*abs(dy);
        }
        else
        {
            x = x+sx;
            y = y+sy;
            Line.u.push_back(x);
            Line.v.push_back(y);
            p = p + 2*abs(dy) - 2*abs(dx);
        }
    }
    return Line;
}


