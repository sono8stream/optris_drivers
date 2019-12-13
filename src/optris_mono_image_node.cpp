/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2016
 *  Technische Hochschule Nürnberg Georg Simon Ohm
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Nuremberg Institute of Technology
 *     Georg Simon Ohm nor the authors names may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christian Pfitzner
 *********************************************************************/

#include <math.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

#include <optris_drivers/ThresholdConfig.h>

#include "libirimager/ImageBuilder.h"

using namespace std;

image_transport::Publisher *_pubThermal;

double _minTemp = -10.0;
double _maxTemp = 40.0;

double _threshold = 40.0;
bool _invert = false;

/**
 * Callback function for receiving thermal data from thermal imager node
 * @param image      image containing raw temperature data
 */
void onThermalDataReceive(const sensor_msgs::ImageConstPtr &image)
{
  static unsigned int frame = 0;

  unsigned short *data = (unsigned short *)&image->data[0];

  sensor_msgs::Image img;
  img.header.frame_id = "thermal_image_view";
  img.height = image->height;
  img.width = image->width;
  img.encoding = "mono8";
  img.step = image->width;
  img.data.resize(img.height * img.step);
  img.header.seq = ++frame;
  img.header.stamp = ros::Time::now();

  // generate binary image from thermal data
  double temp = 0;
  int val = 0;
  for (unsigned int i = 0; i < image->width * image->height; i++)
  {
    temp = (float(data[i * image->step / image->width]) - 1000.0f) / 10.0f;
    //ROS_INFO("%lf",temp);
    temp = max(temp, _minTemp);
    temp = min(temp, _maxTemp);
    val = (int)(0xff * (temp - _minTemp) / (_maxTemp - _minTemp));
    if (_invert)
    {
      img.data[i] = 0xff - val;
    }
    else
    {
      img.data[i] = val;
    }
  }

  _pubThermal->publish(img);
}

/**
 * Callback function for dynamic reconfigure package
 * @param config        configuration
 * @param level         level of configuration
 */
void callback(optris_drivers::ThresholdConfig &config, uint32_t level)
{
  _threshold = config.threshold;
  _invert = config.invert;
}

/**
 * Main routine of optris_binary_image_node.cpp
 * @param argc
 * @param argv
 * @return
 *
 * Usage:
 * rosrun optris_drivers optris_binary_image_node _threshold=20 _invert:=false
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "optris_mono_image_node");

  // set up for dynamic reconfigure server
  dynamic_reconfigure::Server<optris_drivers::ThresholdConfig> server;
  dynamic_reconfigure::Server<optris_drivers::ThresholdConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");

  // parameters for initialization
  _minTemp = -10.0;
  n_.getParam("minTemp", _minTemp);
  _maxTemp = 40.0;
  n_.getParam("maxTemp", _maxTemp);
  ROS_INFO("%lf", _minTemp);
  ROS_INFO("%lf", _maxTemp);

  // init subscribers and publishers
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("/optris/image_mono", 1, onThermalDataReceive);
  image_transport::Publisher pubt = it.advertise("thermal_mono", 1);
  _pubThermal = &pubt;

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
