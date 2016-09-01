#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <std_msgs/Float32.h>

using namespace cv;
using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
	VideoCapture cap(2); //capture video
	
	if ( !cap.isOpened() ) 
	{
		cout << "Cannot access camera." << endl;
		return -1;
	}
	
	int imgh,imgw;
	imgh=480;
	imgw=640;
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, imgw);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgh);
      
  init(argc, argv, "docking_processing_node");
  NodeHandle nh;
  std_msgs::Float32 vertical, horizontal, boxsize;
  Publisher pub_v=nh.advertise<std_msgs::Float32>("docking_vertical",1); //center of image is zero, -> +, ^ +
  Publisher pub_h=nh.advertise<std_msgs::Float32>("docking_horizontal",1);  
  Publisher pub_s=nh.advertise<std_msgs::Float32>("size",1);

  
  time_t start,end;
	
	int iLowH=0;
	int iHighH=10;
		
	int iLowS=50;
	int iHighS=255;
		
	int iLowV=50;
	int iHighV=255;
	
	time(&start);
	int counter=0;
	
	Mat imgOriginal;
	Mat imgHSV;
	Mat imgThresholded;
	vector<vector<Point> > contours;
	int largest_area=0;
    int largest_contour_index=0;


  Rate loop_rate(5);
  while (nh.ok()) {
    
    
    bool bSuccess = cap.read(imgOriginal); // read new frame
		
		if (!bSuccess) 
		{
		cout << "Cannot read frame from video stream." << endl;
		break;
		}
		
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //convert from RGB to HSV

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //threshold
		
		//morphological opening
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		
		//morphological closing
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		
		findContours(imgThresholded, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
		Rect bounding_rect;
		largest_area=0;
		
		for(int i=0; i<contours.size(); i++)
        {
			double a=contourArea(contours[i],false);  //  Find the area of each contour
			if(a>largest_area){
						largest_area=a;
						largest_contour_index=i;                
						bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
							   }
   
         }
		boxsize.data=largest_area;
		time(&end);
		++counter;
		double sec=difftime(end,start);
		double fps=counter/sec;
		
		//move coord sytem into image centre
		horizontal.data=bounding_rect.x+bounding_rect.width/2.0-imgw/2.0;
		vertical.data=-(bounding_rect.y+bounding_rect.height/2.0-imgh/2.0);
		
		vertical.data/=imgh;
		horizontal.data/=imgw;
		
		vertical.data*=2;
		horizontal.data*=2;
		
		//publish offset
		if(largest_area>0)
		{
			pub_v.publish(vertical);
			pub_h.publish(horizontal);
			pub_s.publish(boxsize);
		}
				
		//ostringstream str;
		//str << "FPS: " << fps;
		
		//putText(imgThresholded, str.str(), cvPoint(30,30), CV_FONT_HERSHEY_DUPLEX, 1, cvScalar(255, 255, 0), 1, CV_AA);
		rectangle(imgOriginal, bounding_rect,  Scalar(0,255,0),1, 8,0);  
		//imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image
		cv::waitKey(3);
		
		
		Scalar color(255,255,255);
    

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
