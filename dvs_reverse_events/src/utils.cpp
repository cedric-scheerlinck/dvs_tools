#include "dvs_reverse_events/utils.h"

#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

DECLARE_bool(mirror);
DECLARE_bool(fb);
DECLARE_double(t_minus);

namespace dvs_reverse_events {
namespace utils {
//fds
const std::string OUTPUT_FOLDER = "./stats/";

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag)
{
  if(argc < 2)
  {
    std::cerr << "Error: not enough input arguments.\n"
        "Usage:\n\trosrun dvs_reverse_events reverse_events path_to_bag.bag\n\n"
        "Additional (optional) command-line flags include:\n"
        "\t--n_hot_pix=<number_of_hot_pixels>\n"
        "\t--n_std=<number_of_standard_deviations>\n"
        "\t--no_stats (do not save stats on disk)"<< std::endl;
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);
  std::cout << "Input bag: " << *path_to_input_rosbag << std::endl;
  return true;
}

void load_events(rosbag::View& view, topic_events& events_by_topic)
{
  std::cout << "loading events..." << std::endl;

  std::vector<std::string> seen_topics;
  for(const rosbag::MessageInstance& m : view)
  {
    if(m.getDataType() == "dvs_msgs/EventArray")
    {
      const std::string topic_name = m.getTopic();
      // pointer to the message
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();
      const cv::Size msg_size = cv::Size(s->width, s->height);

      std::vector<dvs_msgs::Event>& event_vec = events_by_topic[topic_name];

      // add to seen topic
      if ( !contains(topic_name, seen_topics) )
      {
        seen_topics.push_back(topic_name);
        std::cout << "added " << topic_name << " to seen_topics" << std::endl;
      }

      for(auto e : s->events)
      {
        // accumulate events without discrimination
        event_vec.push_back(e);
      }
    }
  }

  std::cout << "...done!" << std::endl;
}

void reverse_event_timestamps(topic_events& events_by_topic)
{
  for(auto& topic : events_by_topic)
  {
    const std::string topic_name = topic.first;
    std::vector<dvs_msgs::Event>& event_vec = topic.second;
    if (FLAGS_mirror)
    {
      const double largest_time = event_vec.back().ts.toSec();
      for (auto& e : event_vec)
      {
        double ts = e.ts.toSec();
        ts = -ts + largest_time;
        e.ts = ros::Time(ts);
      }
    }
    else
    {
      const int length = event_vec.size();
      const int midpoint = static_cast<int>(length/2);
      for (int i = 0; i < midpoint; i++)
      {
        const ros::Time tmp = event_vec[i].ts;
        event_vec[i].ts = event_vec[length - i - 1].ts;
        event_vec[length - i].ts = tmp;
      }
    }
  }
}

void flip_polarity(topic_events& events_by_topic)
{
  for(auto& topic : events_by_topic)
  {
    const std::string topic_name = topic.first;
    std::vector<dvs_msgs::Event>& event_vec = topic.second;
    const int length = event_vec.size();
    const int midpoint = static_cast<int>(length/2);
    for (auto& e : event_vec)
    {
      e.polarity = !e.polarity;
    }
  }
}

void write_histogram_image(const std::string filename,
                           const cv::Mat& histogram,
                           const std::vector<cv::Point>& hot_pixels)
{
  cv::Mat display_image;

  if (!hot_pixels.empty())
  {
    cv::Vec3b colour = cv::Vec3b(255, 0, 0);
    cv::Mat local_hist;
    histogram.copyTo(local_hist);
    // create mask
    cv::Mat mask = cv::Mat::zeros(histogram.size(), CV_8UC1);
    for (auto point : hot_pixels)
    {
      mask.at<uint8_t>(point) = 1;
    }
    double max;
    cv::minMaxLoc(local_hist, nullptr, &max);
    local_hist.setTo(max, mask);
    cv::normalize(local_hist, display_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(display_image, display_image, cv::COLORMAP_HOT);
    display_image.setTo(colour, mask);
  }
  else
  {
    cv::normalize(histogram, display_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::applyColorMap(display_image, display_image, cv::COLORMAP_HOT);
  }

  cv::Mat large_image;
  cv::resize(display_image, large_image, cv::Size(), 3, 3, cv::INTER_NEAREST);
  cv::imwrite(filename, large_image);
}

std::string extract_bag_name(const std::string fullname)
{
  int pos = 0;
  int len = fullname.length();
  // go from the back to the first forward- or back-slash.
  for (int i = len; i > 0; i--)
  {
    if (fullname[i] == '/' || fullname[i] == '\\')
    {
      pos = i + 1;
      break;
    }
  }
  int count = 4;
  // now go from there to the first '.'
  for (int i = 0; i < len; i++)
  {
    if (fullname[pos + i] == '.')
    {
      count = i;
      break;
    }
  }
  std::string bag_name = fullname.substr(pos, count);
  return bag_name;
}

void write_all_msgs(rosbag::View& view,
                    topic_events& events_by_topic,
                    rosbag::Bag& output_bag)
{
  constexpr int log_every_n_messages = 10000;
  const uint32_t num_messages = view.size();
  uint32_t message_index = 0;
  std::cout << "Writing..." << std::endl;
  if (!FLAGS_fb)
  {
    // copy paste everything except events from the intput rosbag into the output
    for(rosbag::MessageInstance const m : view)
    {
      write_msg(m, events_by_topic, output_bag);
      if(message_index++ % log_every_n_messages == 0)
      {
        std::cout << "Message: " << message_index << " / " << num_messages << std::endl;
      }
    }
  }
  else
  {
    std::cout << "forward-pass" << std::endl;

    double total_duration;
    std::string event_topic;
    // copy paste everything except events from the intput rosbag into the output
    for(rosbag::MessageInstance const m : view)
    {
      copy_msg(m, output_bag, total_duration);
      if(message_index++ % log_every_n_messages == 0)
      {
        std::cout << "Message: " << message_index << " / " << num_messages << std::endl;
      }
      if(m.getDataType() == "dvs_msgs/EventArray")
      {
        event_topic = m.getTopic();
      }
    }
    std::cout << "backward-pass" << std::endl;
    std::cout << "End time: " << total_duration << std::endl;
    message_index = 0;
    std::vector<dvs_msgs::Event>& event_vec = events_by_topic[event_topic];
    int idx = event_vec.size() - 1;
    for(rosbag::MessageInstance const m : view)
    {

      if(m.getTopic() == event_topic)
      {
        dvs_msgs::EventArrayConstPtr event_array_ptr = m.instantiate<dvs_msgs::EventArray>();
        std::vector<dvs_msgs::Event> local_events;

        for(int i = 0; i < event_array_ptr->events.size(); i++)
        {
          event_vec[idx].ts += ros::Duration(total_duration);
          local_events.push_back(event_vec[idx]);
          idx--;
        }

        if (local_events.size() > 0)
        {
          // Write new event array message to output rosbag

          dvs_msgs::EventArray event_array_msg;
          event_array_msg.events = local_events;
          event_array_msg.width = event_array_ptr->width;
          event_array_msg.height = event_array_ptr->height;
          event_array_msg.header.stamp = local_events.back().ts;

          output_bag.write(event_topic, event_array_msg.header.stamp, event_array_msg);
        }

        if(message_index++ % log_every_n_messages == 0)
        {
          std::cout << "Message: " << message_index << " / " << num_messages << std::endl;
        }
      }
    }
  }

  std::cout << "Message: " << num_messages << " / " << num_messages << std::endl;
  std::cout << "...done!" << std::endl;
}

void copy_msg(const rosbag::MessageInstance& m,
               rosbag::Bag& output_bag,
               double& total_duration)
{
  if(m.getDataType() == "dvs_msgs/EventArray")
  {
    dvs_msgs::EventArrayConstPtr event_array_ptr = m.instantiate<dvs_msgs::EventArray>();
    output_bag.write(m.getTopic(), event_array_ptr->header.stamp - ros::Duration(FLAGS_t_minus), m);
    total_duration = event_array_ptr->header.stamp.toSec() - FLAGS_t_minus;
  }
  else if(m.getDataType() == "sensor_msgs/Image")
  {
    sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
    output_bag.write(m.getTopic(), img_msg->header.stamp - ros::Duration(FLAGS_t_minus), m);
  }
  else if(m.getDataType() == "sensor_msgs/Imu")
  {
    sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
    output_bag.write(m.getTopic(), imu_msg->header.stamp - ros::Duration(FLAGS_t_minus), m);
  }
  else
  {
    output_bag.write(m.getTopic(), m.getTime() - ros::Duration(FLAGS_t_minus), m);
  }
}

void write_msg(const rosbag::MessageInstance& m,
               topic_events& events_by_topic,
               rosbag::Bag& output_bag)
{
  if(m.getDataType() == "dvs_msgs/EventArray")
  {
    const std::string topic_name = m.getTopic();
    std::vector<dvs_msgs::Event>& event_vec = events_by_topic[topic_name];
    dvs_msgs::EventArrayConstPtr event_array_ptr = m.instantiate<dvs_msgs::EventArray>();
    write_event_msg(topic_name, event_array_ptr, event_vec, output_bag);
  }
//  else if(m.getDataType() == "sensor_msgs/Image")
//  {
//    sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
//    output_bag.write(m.getTopic(), img_msg->header.stamp, m);
//  }
//  else if(m.getDataType() == "sensor_msgs/Imu")
//  {
//    sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
//    output_bag.write(m.getTopic(), imu_msg->header.stamp, m);
//  }
//  else
//  {
//    output_bag.write(m.getTopic(), m.getTime(), m);
//  }
}

void write_event_msg(const std::string topic_name,
                     const dvs_msgs::EventArrayConstPtr event_array_ptr,
                     const std::vector<dvs_msgs::Event>& event_vec,
                     rosbag::Bag& output_bag)
{
  std::vector<dvs_msgs::Event> local_events;
  static int idx = event_vec.size() - 1;

  for(int i = 0; i < event_array_ptr->events.size(); i++)
  {
    local_events.push_back(event_vec[idx]);
    idx--;
  }

  if (local_events.size() > 0)
  {
  // Write new event array message to output rosbag

  dvs_msgs::EventArray event_array_msg;
  event_array_msg.events = local_events;
  event_array_msg.width = event_array_ptr->width;
  event_array_msg.height = event_array_ptr->height;
  event_array_msg.header.stamp = local_events.back().ts;

  output_bag.write(topic_name, event_array_msg.header.stamp, event_array_msg);
  }
}

std::string usable_filename(const std::string filename_in)
{
  std::string filename = filename_in;
  std::replace( filename.begin(), filename.end(), '/', '_'); // replace all '/' to '_'
  std::replace( filename.begin(), filename.end(), '\\', '_'); // replace all '\' to '_'
  return filename;
}

}  // namespace utils
}  // namespace dvs_reverse_events
