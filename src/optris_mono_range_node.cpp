#include <math.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

image_transport::Publisher *_pubThermal;

const int HISTO_SIZE = 65536;

bool _invert = false;
int histogram[HISTO_SIZE] = {};
double thres1 = 5;
double thres2 = 95;
cv::Mat img(480, 640, CV_8UC1);

void onThermalDataReceive(const sensor_msgs::ImageConstPtr &image)
{
  try
  {
    cv::Mat input = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO16)->image;
    int all_cnt = input.rows * input.cols;
    for (int i = 0; i < HISTO_SIZE; i++)
    {
      histogram[i] = 0;
    }
    for (int j = 0; j < input.rows; j++)
    {
      unsigned short *src = input.ptr<unsigned short>(j);
      for (int i = 0; i < input.cols; i++)
      {
        histogram[src[i]]++;
      }
    }
    unsigned short min_val = 0;
    unsigned short max_val = 0;
    int now_cnt = 0;
    for (int i = 0; i < HISTO_SIZE; i++)
    {
      now_cnt += histogram[i];
      if (100 * now_cnt > thres1 * all_cnt)
      {
        min_val = i;
        break;
      }
    }
    now_cnt = all_cnt;
    for (int i = 0; i < HISTO_SIZE; i++)
    {
      now_cnt -= histogram[HISTO_SIZE - i - 1];
      if (100 * now_cnt < thres2 * all_cnt)
      {
        max_val = HISTO_SIZE - i - 1;
        break;
      }
    }
    for (int j = 0; j < input.rows; j++)
    {
      unsigned short *src = input.ptr<unsigned short>(j);
      unsigned char *out = img.ptr<unsigned char>(j);
      for (int i = 0; i < input.cols; i++)
      {
        int val = (src[i] - min_val) * 255 / (max_val - min_val);
        val = min(val, 255);
        val = max(val, 0);
        out[i] = val;
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

  n_.getParam("thres1", thres1);
  n_.getParam("thres2", thres2);
  ROS_INFO("%lf", thres1);
  ROS_INFO("%lf", thres2);

  // init subscribers and publishers
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("/adapter/thermal_mono", 1, onThermalDataReceive);
  image_transport::Publisher pubt = it.advertise("thermal_range", 1);
  _pubThermal = &pubt;

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
