#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <time.h>
#include <std_msgs/Float32.h>


static const std::string OPENCV_WINDOW = "Original Kinect IR Stream";

using namespace cv;
using namespace std;
using namespace ros;
using namespace cv_bridge;
using namespace image_transport;


class myMutex {
        public:
                myMutex() {
                        pthread_mutex_init( &m_mutex, NULL );
                }
                void lock() {
                        pthread_mutex_lock( &m_mutex );
                }
                void unlock() {
                        pthread_mutex_unlock( &m_mutex );
                }
        private:
                pthread_mutex_t m_mutex;
};

class MyFreenectDevice : public Freenect::FreenectDevice {
        public:
                MyFreenectDevice(freenect_context *_ctx, int _index)
                        : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_10BIT),
                        m_buffer_rgb(FREENECT_VIDEO_IR_8BIT), m_gamma(2048), m_new_rgb_frame(false),
                        m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
                        rgbMat(Size(640,480), CV_8UC3, Scalar(0)),irMat(Size(640,480), CV_8UC1, Scalar(0)),
                        ownMat(Size(640,480),CV_8UC3,Scalar(0)) {

                        for( unsigned int i = 0 ; i < 2048 ; i++) {
                                float v = i/2048.0;
                                v = std::pow(v, 3)* 6;
                                m_gamma[i] = v*6*256;
                        }

                }


                // Do not call directly even in child
                void VideoCallback(void* _rgb, uint32_t timestamp) {
                        //std::cout << "RGB callback" << std::endl;
                        m_rgb_mutex.lock();
                        //uint8_t* rgb = static_cast<uint8_t*>(_rgb);
                        //rgbMat.data = rgb;
                        if (!irflag)
                                {
                                        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
                                        rgbMat.data = rgb;
                                }
                        else {
                                        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
                                        irMat.data = rgb;
                                }
                        m_new_rgb_frame = true;
                        m_rgb_mutex.unlock();
                };


                // Do not call directly even in child
                void DepthCallback(void* _depth, uint32_t timestamp) {
                        //std::cout << "Depth callback" << std::endl;
                        m_depth_mutex.lock();
                        uint16_t* depth = static_cast<uint16_t*>(_depth);
                        depthMat.data = (uchar*) depth;
                        m_new_depth_frame = true;
                        m_depth_mutex.unlock();
                }

                bool getVideo(Mat& output) {
                        m_rgb_mutex.lock();
                        if(m_new_rgb_frame) {
                                //imshow("raw",rgbMat);
                                if (!irflag) cv::cvtColor(rgbMat, output, CV_RGB2BGR);
                                else irMat.copyTo(output);
                                //cv::cvtColor(rgbMat, output, CV_RGB2BGR);
                                m_new_rgb_frame = false;
                                m_rgb_mutex.unlock();
                                return true;
                        } else {
                                m_rgb_mutex.unlock();
                               return false;
                        }
                }


                bool getDepth(Mat& output) {
                                m_depth_mutex.lock();
                                if(m_new_depth_frame) {
                                        depthMat.copyTo(output);
                                        m_new_depth_frame = false;
                                        m_depth_mutex.unlock();
                                        return true;
                                } else {
                                        m_depth_mutex.unlock();
                                        return false;
                                }
                        }

                bool irflag;
        private:
                std::vector<uint8_t> m_buffer_depth;
                std::vector<uint8_t> m_buffer_rgb;
                std::vector<uint16_t> m_gamma;
                Mat depthMat;
                Mat rgbMat;
                Mat ownMat;
                Mat irMat;
                myMutex m_rgb_mutex;
                myMutex m_depth_mutex;
                bool m_new_rgb_frame;
                bool m_new_depth_frame;
};



int main(int argc, char** argv)
{

        int imgh,imgw;
        imgh=480;
        imgw=640;

        /*cap.set(CV_CAP_PROP_FRAME_WIDTH, imgw);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgh);*/


    Mat depthMat(Size(640,480),CV_16UC1);
        Mat depthf (Size(640,480),CV_8UC1);
        Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
        Mat irMat(Size(640,480),CV_8UC1,Scalar(0));
        Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

        time_t start,end;
        int iLowH=0;
        int iHighH=10;

        int iLowS=30;
        int iHighS=255;

        int iLowV=30;
        int iHighV=255;

        time(&start);
        int counter=0;

        Mat imgOriginal;
        Mat imgHSV;
        Mat imgThresholded;
        vector<vector<Point> > contours;
        int largest_area=0;
    int largest_contour_index=0;

    init(argc, argv, "docking_processing_node_kinect");
    NodeHandle nh;
    std_msgs::Float32 vertical, horizontal, boxsize;
    ros::Publisher pub_v=nh.advertise<std_msgs::Float32>("docking_vertical",1); //center of image is zero, -> +, ^ +
    ros::Publisher pub_h=nh.advertise<std_msgs::Float32>("docking_horizontal",1);
    ros::Publisher pub_s=nh.advertise<std_msgs::Float32>("size",1);

        Freenect::Freenect freenect;
        MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
        device.irflag=false;

        freenect_video_format requested_format(FREENECT_VIDEO_RGB);
        if (*argv[argc-1] == 'i')
                {
                        requested_format = FREENECT_VIDEO_IR_8BIT;
                        device.irflag=true;
                }

        device.setVideoFormat(requested_format);
        device.startVideo();
        //device.startDepth();


  //Rate loop_rate(5); //lmit rate for UDOO quad
  Rate loop_rate(15);
  while (nh.ok()) {

                if (*argv[argc-1] == 'i')
                        {
                        device.getVideo(irMat);
                        
                        Mat m1=irMat;
						double min, max;
						minMaxLoc(irMat, &min, &max);
						imgThresholded=m1>(max-10);
						//imshow("Thresholded Image", imgThresholded);
		
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
						if(largest_area>15)
						{
							pub_v.publish(vertical);
							pub_h.publish(horizontal);
							pub_s.publish(boxsize);
							rectangle(imgOriginal,bounding_rect,Scalar(0,255,0),1,8,0);
						}
								
						ostringstream str;
						str << "FPS: " << fps;
							
						if (*argv[argc-2] != '0') {
						
						putText(imgThresholded, str.str(), cvPoint(30,30), CV_FONT_HERSHEY_DUPLEX, 1, cvScalar(255, 255, 0), 1, CV_AA);
						
						imshow("Contours", imgThresholded); //show the thresholded image
						//imshow("Greyscale Image", imgGray); //show the greyscale image
						//imshow("Hist",m1);
						imshow("IR", irMat); //show the original image
						
						
						
						Scalar color(255,255,255);
							}

                        //imshow("IR", irMat);
                        }
                else
                        {
                        device.getVideo(rgbMat);


                imgOriginal=rgbMat;
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
                if(largest_area>50)
                {
                        pub_v.publish(vertical);
                        pub_h.publish(horizontal);
                        pub_s.publish(boxsize);
                        rectangle(imgOriginal,bounding_rect,Scalar(0,255,0),1,8,0);
                }

                ostringstream str;
                str << "FPS: " << fps;
                
                if (*argv[argc-2] != '0') {

                putText(imgThresholded, str.str(), cvPoint(30,30), CV_FONT_HERSHEY_DUPLEX, 1, cvScalar(255, 255, 0), 1, CV_AA);
                //rectangle(imgOriginal, bounding_rect,  Scalar(0,255,0),1, 8,0);  
                imshow("Thresholded Image", imgThresholded); //show the thresholded image
                imshow("Original", imgOriginal); //show the original image

                Scalar color(255,255,255);
        }
	}

                char k = cvWaitKey(5);
                if( k == 27 ){
                        cvDestroyWindow("Kinect");
                        cvDestroyWindow("IR");
                        break;
                }

    ros::spinOnce();
    loop_rate.sleep();
  }

  device.stopVideo();
  //device.stopDepth();
}

