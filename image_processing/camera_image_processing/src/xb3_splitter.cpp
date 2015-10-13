#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <ros/ros.h>

class xb3ImageDeinterlacer {
  private:
    ros::NodeHandle leftNh,centerNh,rightNh;
    sensor_msgs::CameraInfo leftInfo,centerInfo,rightInfo;
    image_transport::CameraPublisher pLeftImage,pRightImage,pCenterImage;

  public:
    xb3ImageDeinterlacer(ros::NodeHandle n1, ros::NodeHandle n2, ros::NodeHandle n3) {
    leftNh=n1; centerNh=n2; rightNh=n3;
    image_transport::ImageTransport itLeft(leftNh);
    pLeftImage = itLeft.advertiseCamera("image", 1000);
    camera_info_manager::CameraInfoManager leftInfoMgr(leftNh);
    leftInfoMgr.loadCameraInfo("package://xb3_driver/cal_left.yaml");
    leftInfo = leftInfoMgr.getCameraInfo();
    leftInfo.header.frame_id = "/camera_frame";

    image_transport::ImageTransport itCenter(centerNh);
    pCenterImage = itCenter.advertiseCamera("image", 1000);
    camera_info_manager::CameraInfoManager centerInfoMgr(centerNh);
    centerInfoMgr.loadCameraInfo("package://xb3_driver/cal_center.yaml");
    centerInfo = centerInfoMgr.getCameraInfo();
    centerInfo.header.frame_id = "/camera_frame";

    image_transport::ImageTransport itRight(rightNh);
    pRightImage = itRight.advertiseCamera("image", 1000);
    camera_info_manager::CameraInfoManager rightInfoMgr(rightNh);
    rightInfoMgr.loadCameraInfo("package://xb3_driver/cal_right.yaml");
    rightInfo = rightInfoMgr.getCameraInfo();
    rightInfo.header.frame_id = "/camera_frame";
  }

  void imageCallback(const sensor_msgs::ImagePtr& img){
    sensor_msgs::Image left,right,center;
    for(unsigned int i=0; i<img->data.size(); i+=3) {
      right.data.push_back(img->data[i]);
      center.data.push_back(img->data[i+1]);
      left.data.push_back(img->data[i+2]);
    }

    leftInfo.header.stamp=left.header.stamp=img->header.stamp;
    left.header.frame_id = "/camera_frame";
    left.height=leftInfo.height; 
    left.width=leftInfo.width; 
    left.step=left.width;
    left.encoding="mono8";
    pLeftImage.publish(left,leftInfo);

    centerInfo.header.stamp=center.header.stamp=img->header.stamp;
    center.header.frame_id = "/camera_frame";
    center.height=centerInfo.height; center.width=centerInfo.width; center.step=left.width; center.encoding="mono8";
    pCenterImage.publish(center,centerInfo);

    rightInfo.header.stamp=right.header.stamp=img->header.stamp;
    right.header.frame_id = "/camera_frame";
    right.height=rightInfo.height; right.width=rightInfo.width; right.step=left.width; right.encoding="mono8";
    pRightImage.publish(right,rightInfo);
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "xb3_deinterlacer");
  ros::NodeHandle n;
  ros::NodeHandle npLeft("xb3_left"),npRight("xb3_right"),npCenter("xb3_center");
  xb3ImageDeinterlacer imgDI(npLeft,npCenter,npRight);
  ros::Subscriber sub1 = n.subscribe("/camera/image_raw", 1000, &xb3ImageDeinterlacer::imageCallback, &imgDI);
  ros::spin();
  return 0;
}
