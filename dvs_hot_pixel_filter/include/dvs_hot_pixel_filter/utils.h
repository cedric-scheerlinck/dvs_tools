#pragma once

#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <vector>

#include <dvs_msgs/EventArray.h>

namespace rosbag
{
class Bag;
class MessageInstance;
class View;
}

namespace dvs_hot_pixel_filter
{
namespace utils
{

typedef std::unordered_map<std::string, cv::Mat> topic_mats;
typedef std::unordered_map<std::string, std::vector<cv::Point>> topic_points;

template <typename T>
bool contains(T element, std::vector<T> my_vector)
{
  // returns true if my_vector contains element, otherwise false.
  for (auto topic_name : my_vector)
  {
    if (element == topic_name)
    {
      return true;
    }
  }
  return false;
}

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag);

std::string extract_bag_name(const std::string fullname);

std::string usable_filename(const std::string filename_in);

void write_histogram_image(const std::string filename,
                           const cv::Mat& histogram,
                           const std::vector<cv::Point>& hot_pixels
                           = std::vector<cv::Point>());

void build_histograms(rosbag::View& view,
                      topic_mats& histograms);

void detect_hot_pixels(const topic_mats& histograms_by_topic,
                       const double& num_std_devs,
                       const int num_hot_pixels,
                       topic_points& hot_pixels_by_topic);

void hot_pixels_by_threshold(const cv::Mat& histogram,
                             const double& threshold,
                             std::vector<cv::Point>& hot_pixels);

void hot_pixels_by_ranking(const cv::Mat& histogram,
                           const double& num_hot_pixels,
                           std::vector<cv::Point>& hot_pixels);

void find_threshold(const cv::Mat& histogram,
                    const double num_std_devs,
                    double& threshold);

void write_all_msgs(rosbag::View& view,
                    topic_points& hot_pixels_by_topic,
                    rosbag::Bag& output_bag);

void write_event_msg(const std::string topic_name,
                     const dvs_msgs::EventArrayConstPtr event_array_ptr,
                     const std::vector<cv::Point>& hot_pixels,
                     rosbag::Bag& output_bag);

void write_msg(const rosbag::MessageInstance& m,
               topic_points& hot_pixels_topic,
               rosbag::Bag& output_bag);

void write_hot_pixels(const std::string filename,
                      const std::vector<cv::Point>& hot_pixels);

void save_stats(const std::string bag_name,
                 const std::string topic_name,
                 const cv::Mat& histogram,
                 const std::vector<cv::Point>& hot_pixels,
                 const bool one_topic);




}  // namespace utils
}  // namespace dvs_hot_pixel_filter
