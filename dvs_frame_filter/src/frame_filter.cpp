#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>

#define foreach BOOST_FOREACH

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag)
{
  if(argc < 2)
  {
    std::cerr << "Not enough arguments" << std::endl;
    std::cerr << "Usage: rosrun dvs_rosbag_stats dvs_rosbag_stats path_to_bag.bag";
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);

  return true;
}

bool compute_stats(const std::string path_to_input_rosbag,
                   int& num_events,
                   int& num_frames,
                   double& duration)
{
  std::cout << "Processing: " << path_to_input_rosbag << std::endl;

  auto const pos = path_to_input_rosbag.find_last_of('/');
  const std::string output_dir = path_to_input_rosbag.substr(0, pos + 1) + "stats/";
  const std::string output_filename = path_to_input_rosbag.substr(
      pos + 1, path_to_input_rosbag.length() - (pos + 1) - 4) + ".txt";
  const std::string path_to_output = output_dir + output_filename;
  boost::filesystem::create_directories(output_dir);

  rosbag::Bag input_bag;
  try
  {
    input_bag.open(path_to_input_rosbag, rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    std::cerr << "Error: could not open rosbag: " << path_to_input_rosbag << std::endl;
    return false;
  }

  rosbag::View view(input_bag);

  std::unordered_map<std::string, std::vector<dvs_msgs::Event>> event_packets_for_each_event_topic;

  const uint32_t num_messages = view.size();
  uint32_t message_index = 0;

  int num_events_tmp = 0;
  int num_frames_tmp = 0;
  double start_time;
  double end_time = 0;
  bool first_msg = true;

  foreach(rosbag::MessageInstance const m, view)
  {
    if (m.getDataType() == "dvs_msgs/EventArray")
    {

      std::vector<dvs_msgs::Event>& events = event_packets_for_each_event_topic[m.getTopic()];
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();
      num_events_tmp += s->events.size();
      if (first_msg)
      {
        start_time = s->events.front().ts.toSec();
        first_msg = false;
      }
      end_time = std::max(s->events.back().ts.toSec(), end_time);
    }
    else if (m.getDataType() == "sensor_msgs/Image")
    {
      num_frames_tmp += 1;
    }
  }

  input_bag.close();

  num_events = num_events_tmp;
  num_frames = num_frames_tmp;
  duration = end_time - start_time;

  return true;
}

bool write_stats(const std::string path_to_output,
                 const int& num_events,
                 const int& num_frames,
                 const double& duration)
{

  std::ofstream stats_file;
  stats_file.open(path_to_output);
//  stats_file << "Number of events: " << num_events << '\n';
//  stats_file << "Number of frames: " << num_frames << '\n';
//  stats_file << "Total duration (s): " << duration << '\n';
  stats_file << num_events << ", " << num_frames << ", " << duration << std::endl;
  stats_file.close();
  return true;
}

bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char* argv[])
{
  std::string path_to_input_rosbag;
  int max_num_events_per_packet;
  ros::Duration max_duration_event_packet;

  if (!parse_arguments(argc, argv, &path_to_input_rosbag))
  {
    return -1;
  }

  int num_events;
  int num_frames;
  double duration;

  if (!compute_stats(path_to_input_rosbag,
                    num_events,
                    num_frames,
                    duration))
  {
    return -1;
  }

  auto const pos = path_to_input_rosbag.find_last_of('/');
  const std::string output_dir = path_to_input_rosbag.substr(0, pos + 1) + "stats/";
  const std::string output_filename = path_to_input_rosbag.substr(
      pos + 1, path_to_input_rosbag.length() - (pos + 1) - 4) + ".txt";
  const std::string path_to_output = output_dir + output_filename;
  boost::filesystem::create_directories(output_dir);
  write_stats(path_to_output,
              num_events,
              num_frames,
              duration);
  return 0;
}
