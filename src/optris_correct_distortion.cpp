#include <math.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

image_transport::Publisher *_pubThermal;
double k1 = 1.05791597e-06;
double k2 = 5.26154073e-14;
double k3 = 3.41991153e-06;
double k4 = 3.27612688e-13;
double p1 = -4.30326545e-06;
double p2 = -4.60648477e-06;
cv::Mat img(560, 882, CV_8UC1);

void onThermalDataReceive(const sensor_msgs::ImageConstPtr &image)
{
  try
  {
    cv::Mat input = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8)->image;
    for (int i = 0; i < img.rows; i++)
    {
      unsigned char *out = img.ptr<unsigned char>(i);
      for (int j = 0; j < img.cols; j++)
      {
        int x1 = j - img.cols / 2;
        int y1 = img.rows / 2 - i;
        long long r_2 = x1 * x1 + y1 * y1;
        long long r_4 = r_2 * r_2;
        double x2 = x1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4) + 2 * p1 * x1 * y1 + p2 * (r_2 + 2 * x1 * x1);
        double y2 = y1 * (1 + k1 * r_2 + k2 * r_4) / (1 + k3 * r_2 + k4 * r_4) + 2 * p2 * x1 * y1 + p1 * (r_2 + 2 * y1 * y1);
        int ii = input.rows / 2 - (int)round(y2);
        int jj = input.cols / 2 + (int)round(x2);
        if (0 <= ii && ii < input.rows && 0 <= jj && jj < input.cols)
        {
          out[j] = input.at<unsigned char>(ii, jj);
        }
      }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    _pubThermal->publish(msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_beidge exception: %s", e.what());
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "optris_correct_distortion");

  ros::NodeHandle n_("~");

  // init subscribers and publishers
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("/adapter/thermal_mono", 1, onThermalDataReceive);
  image_transport::Publisher pubt = it.advertise("thermal_corrected", 1);
  _pubThermal = &pubt;

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
