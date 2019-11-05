#pragma once

#include <string>
#include <stdio.h>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_event_calibration/dvs_event_calibrationConfig.h>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

// google logging
#include <glog/logging.h>
#include <gflags/gflags.h>

namespace dvs_event_calibration
{

class Calibrator
{
public:
  Calibrator(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void set_parameters();
  void save_calibration(std::string save_dir);
  virtual ~Calibrator();

private:
  ros::NodeHandle nh_;

  void initialise_image_states(const uint32_t& rows, const uint32_t& columns);

  cv::Mat on_count_mat_;
  cv::Mat off_count_mat_;

  bool initialised_;
};

} // namespace
