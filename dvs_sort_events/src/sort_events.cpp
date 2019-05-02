#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>
#include <chrono>

#include <gflags/gflags.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>

#include "../include/dvs_sort_events/utils.h"

constexpr int INITIAL_MAX_EVENTS = -1;
constexpr double INITIAL_MAX_DURATION_MS = 1e6;

DEFINE_int32(max_events, INITIAL_MAX_EVENTS, "Maximum number of events per output message");
DEFINE_double(max_duration_ms, INITIAL_MAX_DURATION_MS, "Maximum duration of output message in milliseconds");

int main(int argc, char* argv[])
{
  std::string path_to_input_rosbag;
  // parse my arguments first before google gflags
  if (!dvs_sort_events::utils::parse_arguments(argc,
                       argv,
                       &path_to_input_rosbag
                       ))
  {
    return -1;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string path_to_output_rosbag = path_to_input_rosbag + ".sorted";
  dvs_sort_events::utils::topic_eventArray events_by_topic;

  if (!dvs_sort_events::utils::load_events_from_bag(path_to_input_rosbag, events_by_topic))
  {
    return -1;
  }

  std::cout << "Sorting... ";

  auto start = std::chrono::high_resolution_clock::now();
  dvs_sort_events::utils::sort_events(events_by_topic);
  auto stop = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed_seconds = stop-start;

  std::cout << "took " << elapsed_seconds.count() << "s." << std::endl;

  std::cout << "Writing... ";

  const double max_duration_ms = (FLAGS_max_events == INITIAL_MAX_EVENTS
                                  && FLAGS_max_duration_ms == INITIAL_MAX_DURATION_MS)
                                  ? 30 : FLAGS_max_duration_ms; // limit default to 30ms if no flags passed

  dvs_sort_events::utils::write_events_to_bag(events_by_topic,
                                              FLAGS_max_events,
                                              max_duration_ms/1e3,
                                              path_to_output_rosbag);
  std::cout << "done!" << std::endl;


  return 0;
}
