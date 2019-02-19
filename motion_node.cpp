#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>


static const std::string OPENCV_WINDOW = "Image window";
int choice = 2;

class ImageConverter
{
  ros::NodeHandle motion_node;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_prev; 
  int prev_set;
public:
  ImageConverter() : it_(motion_node)
  {
    image_sub_ = it_.subscribe("camera/visible/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    prev_set = 0;
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat gcur,gprev;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (prev_set == 0)
    {
    	cv_prev = cv_ptr;
    	prev_set = 1;
    }

	cv::Mat FOM(cv_ptr->image.size(), CV_32FC2);
	cv::Mat motion(cv_ptr->image.cols,cv_ptr->image.rows, CV_8UC1, cv::Scalar(0));
    
	if (choice == 2)
	{
		std::vector<cv::Mat> channels(2);
		cv::cvtColor(cv_ptr->image,gcur,CV_RGB2GRAY);
		cv::cvtColor(cv_prev->image,gprev,CV_RGB2GRAY);
		cv::calcOpticalFlowFarneback(gprev,gcur,FOM,0.5, 3, 12, 3, 5, 1.2, 0);
		split(FOM, channels);
		motion = (cv::abs(channels[0]) + cv::abs(channels[1])) > 0.1;
	}
	else if (choice == 3)
	{
	}
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, motion);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
    cv_prev = cv_ptr;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}