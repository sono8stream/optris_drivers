#include <math.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

image_transport::Publisher *_pubThermal;

double _minTemp = -10.0;
double _maxTemp = 40.0;

double _threshold = 40.0;
bool _invert = false;
cv::Mat img(480, 640, CV_8UC1);

void onThermalDataReceive(const sensor_msgs::ImageConstPtr &image)
{
  try
  {
    cv::Mat input = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO16)->image;
    for (int j = 0; j < input.rows; j++)
    {
      unsigned short *src = input.ptr<unsigned short>(j);
      unsigned char *out = img.ptr<unsigned char>(j);
      ROS_INFO("%d", sizeof(src[0]));
      for (int i = 0; i < input.cols; i++)
      {
        //ROS_INFO("%d", src[i]);
        double temperature = (src[i] - 1000.0) / 10.0f;
        temperature = min(temperature, _maxTemp);
        temperature = max(temperature, _minTemp);
        int val = (int)((temperature - _minTemp) * 0xFF / (_maxTemp - _minTemp));
        if (_invert)
        {
          out[i] = 0xFF - val;
        }
        else
        {
          out[i] = val;
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
  ros::init(argc, argv, "optris_mono_image_node");

  ros::NodeHandle n_("~");

  // parameters for initialization
  _minTemp = -10.0;
  n_.getParam("minTemp", _minTemp);
  _maxTemp = 40.0;
  n_.getParam("maxTemp", _maxTemp);
  ROS_INFO("%lf", _minTemp);
  ROS_INFO("%lf", _maxTemp);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("/adapter/thermal_mono", 1, onThermalDataReceive);
  image_transport::Publisher pubt = it.advertise("thermal_fix_range", 1);
  _pubThermal = &pubt;

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
