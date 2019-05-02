#include "../include/dvs_sort_events/utils.h"

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <gflags/gflags.h>
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#define foreach BOOST_FOREACH

namespace dvs_sort_events {
namespace utils {
//const std::string OUTPUT_FOLDER = "./stats/";

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag
                     )
{
  constexpr int expected_num_arguments = 2;
  if(argc < expected_num_arguments)
  {
    std::cerr << "Not enough arguments, "<< argc << " given, " << expected_num_arguments << " expected." << std::endl;
    std::cerr << "Usage: rosrun dvs_crop_bag dvs_crop_bag path_to_bag.bag" << std::endl;
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);

  return true;
}

bool load_events_from_bag(std::string path_to_input_rosbag,
                          topic_eventArray& events_by_topic)
{
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

  load_events(view, events_by_topic);

  input_bag.close();

  return true;
}

void load_events(rosbag::View& view, topic_eventArray& events_by_topic)
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

      dvs_msgs::EventArray& event_array = events_by_topic[topic_name];

      // add to seen topic
      if ( !contains(topic_name, seen_topics) )
      {
        seen_topics.push_back(topic_name);
        event_array.width = s->width;
        event_array.height = s->height;
        std::cout << "added " << topic_name << " to seen_topics" << std::endl;
      }

      for(auto e : s->events)
      {
        // accumulate events without discrimination
        event_array.events.push_back(e);
      }
    }
  }

  std::cout << "...done!" << std::endl;
}

void sort_events(topic_eventArray& events_by_topic)
{
  for (auto& iter: events_by_topic)
  {
    auto& event_array = iter.second;
    std::sort(event_array.events.begin(), event_array.events.end(), is_timestamp_in_order);
  }

}

bool is_timestamp_in_order(dvs_msgs::Event first, dvs_msgs::Event second)
{
  return first.ts.toSec() < second.ts.toSec();
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

void write_events_to_bag(topic_eventArray& events_by_topic,
                         const int& max_num_events_per_packet,
                         const double& max_duration_event_packet,
                         std::string path_to_output_rosbag)
{
  rosbag::Bag output_bag;
  output_bag.open(path_to_output_rosbag, rosbag::bagmode::Write);

  for (auto& iter: events_by_topic)
  {
    std::string topic = iter.first;
    dvs_msgs::EventArray& event_array_in = iter.second;
    std::vector<dvs_msgs::Event> events_out;
    for(auto e : event_array_in.events)
    {
      events_out.push_back(e);
      if(events_out.size() == max_num_events_per_packet
         || (events_out.back().ts.toSec() - events_out.front().ts.toSec()) >= max_duration_event_packet)
      {
        // Write new event array message to output rosbag
        dvs_msgs::EventArray event_array_msg;
        event_array_msg.events = events_out;
        event_array_msg.width = event_array_in.width;
        event_array_msg.height = event_array_in.height;
        event_array_msg.header.stamp = events_out.back().ts;

        output_bag.write(topic, event_array_msg.header.stamp, event_array_msg);

        // Start new packet of events
        events_out.clear();
      }
    }
  }
  output_bag.close();
}


dvs_msgs::EventArray new_event_msg(rosbag::MessageInstance const& m,
                                   const ros::Duration& duration_to_subtract
                                   )
{
  dvs_msgs::EventArray event_array_msg;

  std::vector<dvs_msgs::Event> events;
  dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();
  for(auto e : s->events)
  {
    double new_ts = e.ts.toSec() - duration_to_subtract.toSec();
    if (new_ts > 0.001) // not too close to 'zero'
    {
      e.ts = ros::Time(new_ts);
      events.push_back(e);
    }
  }

  event_array_msg.events = events;
  event_array_msg.width = s->width;
  event_array_msg.height = s->height;
  event_array_msg.header.stamp = s->header.stamp - duration_to_subtract;

  return event_array_msg;
}

std::string usable_filename(const std::string filename_in)
{
  std::string filename = filename_in;
  std::replace( filename.begin(), filename.end(), '/', '_'); // replace all '/' to '_'
  std::replace( filename.begin(), filename.end(), '\\', '_'); // replace all '\' to '_'
  return filename;
}

}  // namespace utils
}  // namespace dvs_sort_events
