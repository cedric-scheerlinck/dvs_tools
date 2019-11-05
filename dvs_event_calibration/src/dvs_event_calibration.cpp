#include "dvs_event_calibration/dvs_event_calibration.h"
#include <cstdlib>
#include <std_msgs/Float32.h>
#include <glog/logging.h>
#include "dvs_event_calibration/utils.h"
#include <string>

namespace dvs_event_calibration
{

Calibrator::Calibrator(ros::NodeHandle & nh, ros::NodeHandle nh_private)
{
  // flags and counters
  initialised_ = false;
}

Calibrator::~Calibrator()
{
}

void Calibrator::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // initialisation only to be performed once at the beginning
  if (!initialised_)
  {
    initialise_image_states(msg->height, msg->width);
  }

  // count events per pixels with polarity
  for (auto e : msg->events)
  {
    if (e.polarity)
    {
      on_count_mat_.at<double>(e.y, e.x) = on_count_mat_.at<double>(e.y, e.x) + 1;
    }
    else
    {
      off_count_mat_.at<double>(e.y, e.x) = off_count_mat_.at<double>(e.y, e.x) + 1;
    }
  }
}

void Calibrator::initialise_image_states(const uint32_t& rows, const uint32_t& columns)
{
  on_count_mat_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  off_count_mat_ = cv::Mat::zeros(rows, columns, CV_64FC1);
  initialised_ = true;
  VLOG(2) << "Initialised!";
}

void Calibrator::save_calibration(std::string save_dir)
{
  constexpr double BASE_ON = 0.1;
  constexpr double BASE_OFF = -BASE_ON;
  cv::Mat c_on_mat;
  cv::Mat c_off_mat;
  const double mean_count = cv::mean(on_count_mat_)[0] + cv::mean(off_count_mat_)[0];
  cv::divide(BASE_ON*mean_count/2, on_count_mat_ + 1e-6, c_on_mat);
  cv::divide(BASE_OFF*mean_count/2, off_count_mat_ + 1e-6, c_off_mat);

  const double difference_factor = 5.0;
  cv::Mat mask_on = c_on_mat > difference_factor*BASE_ON;
  cv::Mat mask_off = c_off_mat < difference_factor*BASE_OFF;
  c_on_mat.setTo(difference_factor*BASE_ON, mask_on);
  c_off_mat.setTo(difference_factor*BASE_OFF, mask_off);

  VLOG(1) << "Total events: " << (cv::sum(on_count_mat_)[0] + cv::sum(off_count_mat_)[0])/1e6 << "M";
  VLOG(1) << "Mean event count at each pixel is: " << mean_count/1e3 << "k";
  VLOG(1) << cv::sum(mask_on)[0] + cv::sum(mask_off)[0] << " pixel(s) had to be truncated";
  VLOG(1) << "Mean ON / OFF: " << cv::mean(c_on_mat)[0] << " / " << cv::mean(c_off_mat)[0];
  VLOG(1) << cv::sum(mask_on)[0] + cv::sum(mask_off)[0] << " pixel(s) had to be truncated";

  cv::FileStorage on_file(save_dir + "/contrast_threshold_on.xml", cv::FileStorage::WRITE);
  on_file << "c" << c_on_mat;
  on_file.release();
  cv::FileStorage off_file(save_dir + "/contrast_threshold_off.xml", cv::FileStorage::WRITE);
  off_file << "c" << c_off_mat;
  off_file.release();

  // sanity check
  cv::Mat c_on_mat_tmp;
  cv::FileStorage on_file_tmp(save_dir + "/contrast_threshold_on.xml", cv::FileStorage::READ);
  on_file_tmp["c"] >> c_on_mat_tmp;
  on_file_tmp.release();
  cv::Mat c_off_mat_tmp;
  cv::FileStorage off_file_tmp(save_dir + "/contrast_threshold_off.xml", cv::FileStorage::READ);
  off_file_tmp["c"] >> c_off_mat_tmp;
  off_file_tmp.release();

  cv::imwrite(save_dir + "/vis_on.bmp", c_on_mat_tmp/BASE_ON*127);
  cv::imwrite(save_dir + "/vis_off.bmp", c_off_mat_tmp/BASE_OFF*127);
}

} // namespace
