// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#include "dvs_binder/binder.h"
#include <std_msgs/Float32.h>
#include <vector>

namespace dvs_binder {

Binder::Binder(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  got_camera_info_ = false;

  // get parameters of display method
  std::string bind_method_str;
  nh_private.param<std::string>("bind_method", bind_method_str, "");
  bind_method_ = (bind_method_str == std::string("grayscale")) ? GRAYSCALE : BINDED;

  // setup subscribers and publishers
  event_sub_ = nh_.subscribe("events", 1, &Binder::eventsCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &Binder::cameraInfoCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_sub_ = it_.subscribe("image", 1, &Binder::imageCallback, this);
  image_pub_ = it_.advertise("dvs_binder", 1);
}

Binder::~Binder()
{
  image_pub_.shutdown();
}

void Binder::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info_ = true;

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (unsigned int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];
}

void Binder::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  // To check if this is true
  last_image_ = cv_ptr->image;

  if (!used_last_image_)
  {
    cv_bridge::CvImage cv_image;
    last_image_.copyTo(cv_image.image);
    cv_image.encoding = "mono8";
    std::cout << "publish image from callback" << std::endl;
    image_pub_.publish(cv_image.toImageMsg());
  }
  used_last_image_ = false;
}

void Binder::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;
    if (msg->events.size() > 0)
    {
      cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
    }

    if (bind_method_ == BINDED)
    {
      cv_image.encoding = "bg8";
      std::vector<cv::Mat> channels(2);

      // define image channels
      if (last_image_.rows == msg->height && last_image_.cols == msg->width)
      {
        last_image_.copyTo(channels[0]);
      }
      else
      {
        channels[0] = cv::Mat(msg->height, msg->width, CV_8U);
        channels[0] = cv::Scalar(0);
      }

      // define event channels
      channels[1] = cv::Mat(msg->height, msg->width, CV_8U);
      channels[1] = cv::Scalar(threshold_);

      unsigned int num_events = (bind_max_events_ < msg->events.size()) ? bind_max_events_ : msg->events.size();

      for (unsigned int i = 0; i < num_events; ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        if (msg->events[i].polarity == 1 && channels[1].at<uint8_t>(cv::Point(x, y)) < threshold_*2)
          channels[1].at<uint8_t>(cv::Point(x, y))++;
        else if (channels[1].at<uint8_t>(cv::Point(x, y)) > 0)
          channels[1].at<uint8_t>(cv::Point(x, y))--;
      }

      cv::merge(channels, cv_image.image);
    }
    else
    {
      // creates only dvs image
      cv_image.encoding = "mono8";
      cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
      cv_image.image = cv::Scalar(threshold_);

      // count events per pixels with polarity
      unsigned int num_events = (bind_max_events_ < msg->events.size()) ? bind_max_events_ : msg->events.size();
      for (unsigned int i = 0; i < num_events; ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        if (msg->events[i].polarity == 1 && cv_image.image.at<uint8_t>(cv::Point(x, y)) < threshold_*2)
          cv_image.image.at<uint8_t>(cv::Point(x, y))++;
        else if (cv_image.image.at<uint8_t>(cv::Point(x, y)) > 0)
          cv_image.image.at<uint8_t>(cv::Point(x, y))--;
      }
    }

    image_pub_.publish(cv_image.toImageMsg());
    // if (got_camera_info_)
    // {
    //   // publish undistorted image when camera info is available
    //   cv_bridge::CvImage cv_image2;
    //   cv_image2.encoding = cv_image.encoding;
    //   cv::undistort(cv_image.image, cv_image2.image, camera_matrix_, dist_coeffs_);
    //   image_pub_.publish(cv_image.toImageMsg());
    // }
    // else
    // {
    //   image_pub_.publish(cv_image.toImageMsg());
    // }
  }
}

} // namespace
