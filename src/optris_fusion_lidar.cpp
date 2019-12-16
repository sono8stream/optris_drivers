#include <math.h>
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher points_pub;
image_transport::Publisher *_pubThermal;
pcl::PointCloud<pcl::PointXYZI> cloudXYZI;

void onThermalReceive(const ImageConstPtr &image)
{
  try
  {
    cv::Mat input = cv_bridge::toCvCopy(image, image_encodings::MONO8)->image;
    cv::Mat img(input.rows, input.cols, CV_8UC3);
    for (int i = 0; i < img.rows; i++)
    {
      cv::Vec3b *out = img.ptr<cv::Vec3b>(i);
      for (int j = 0; j < img.cols; j++)
      {
        unsigned char val = input.at<unsigned char>(i, j);
        out[j][0] = val;
        out[j][1] = 0;
        out[j][2] = 0;
        //out[j][1] = val;
        //out[j][2] = val;
      }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr extracteds(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(cloudXYZI, *extracteds);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int i = 0; i < cloudXYZI.size(); i++)
    {
      float x = cloudXYZI.points[i].y;
      float y = -cloudXYZI.points[i].x;
      float z = cloudXYZI.points[i].z - 0.1;
      float f = img.cols / 2 * 1.01f;
      if (y < 0)
      {
        continue;
      }
      int u = (int)(img.cols / 2 + f * x / y);
      int v = (int)(img.rows / 2 - f * z / y);
      if (0 <= u && u < img.cols && 0 <= v && v < img.rows)
      {
        img.at<cv::Vec3b>(v, u)[2] = 255;
        inliers->indices.push_back(i);
      }
    }
    ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    _pubThermal->publish(msg);

    pcl::ExtractIndices<pcl::PointXYZI> extractor;
    extractor.setInputCloud(extracteds);
    extractor.setIndices(inliers);
    extractor.setNegative(false);
    extractor.filter(*extracteds);

    pcl::PCLPointCloud2 cloud_out;
    pcl::toPCLPointCloud2(*extracteds, cloud_out);
    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_out, output);

    // Publish the data
    points_pub.publish(output);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_beidge exception: %s", e.what());
  }
}

void onLiDARReceive(const PointCloud2ConstPtr &cloud_msg)
{

  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::fromPCLPointCloud2(*cloudPtr, cloudXYZI);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "optris_fusion_lidar");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");

  // init subscribers and publishers
  ros::NodeHandle n;
  ros::Subscriber thermal_sub = n.subscribe("/thermal_range", 1, onThermalReceive);
  ros::Subscriber lidar_sub = n.subscribe("/adapter/lidar_points", 1, onLiDARReceive);
  image_transport::ImageTransport it(n);
  image_transport::Publisher pubt = it.advertise("thermal_fusion", 1);
  _pubThermal = &pubt;
  points_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
