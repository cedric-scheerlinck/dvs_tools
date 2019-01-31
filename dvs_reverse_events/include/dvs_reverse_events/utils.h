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

namespace dvs_reverse_events
{
namespace utils
{

typedef std::unordered_map<std::string, cv::Mat> topic_mats;
typedef std::unordered_map<std::string, std::vector<cv::Point>> topic_points;
typedef std::unordered_map<std::string, std::vector<dvs_msgs::Event>> topic_events;

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

void load_events(rosbag::View& view, topic_events& events_by_topic);

void reverse_event_timestamps(topic_events& events_by_topic);

void flip_polarity(topic_events& events_by_topic);

void write_all_msgs(rosbag::View& view,
                    topic_events& events_by_topic,
                    rosbag::Bag& output_bag);

void copy_msg(const rosbag::MessageInstance& m,
               rosbag::Bag& ,
               double& total_duration);

void write_msg(const rosbag::MessageInstance& m,
               topic_events& events_by_topic,
               rosbag::Bag& output_bag);

void write_event_msg(const std::string topic_name,
                     const dvs_msgs::EventArrayConstPtr event_array_ptr,
                     const std::vector<dvs_msgs::Event>& event_vec,
                     rosbag::Bag& output_bag);

void write_hot_pixels(const std::string filename,
                      const std::vector<cv::Point>& hot_pixels);

}  // namespace utils
}  // namespace dvs_reverse_events
