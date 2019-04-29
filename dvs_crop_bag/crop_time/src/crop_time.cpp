#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>

#include <gflags/gflags.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>

#define foreach BOOST_FOREACH

DEFINE_double(t1s, -1, "timestamp 1 start");
DEFINE_double(t1e, -1, "timestamp 1 end");
DEFINE_double(t2s, -1, "timestamp 2 start");
DEFINE_double(t2e, -1, "timestamp 2 end");
DEFINE_double(t3s, -1, "timestamp 3 start");
DEFINE_double(t3e, -1, "timestamp 3 end");

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag
                     )
{
  constexpr int expected_num_arguments = 3;
  if(argc < expected_num_arguments)
  {
    std::cerr << "Not enough arguments, "<< argc << " given, " << expected_num_arguments << " expected." << std::endl;
    std::cerr << "Usage: rosrun dvs_crop_bag dvs_crop_bag path_to_bag.bag" << std::endl;
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);

  return true;
}

bool time_within_crop(const ros::Time& timestamp)
{
  static const double first_timestamp = timestamp.toSec(); // initialised only once
  const double t_now = timestamp.toSec() - first_timestamp;

  bool in_t1 = t_now > FLAGS_t1s && t_now < FLAGS_t1e;
  bool in_t2 = t_now > FLAGS_t2s && t_now < FLAGS_t2e;
  bool in_t3 = t_now > FLAGS_t3s && t_now < FLAGS_t3e;
  return in_t1 || in_t2 || in_t3;
}

int main(int argc, char* argv[])
{
  std::string path_to_input_rosbag;
  // parse my arguments first before google gflags
  if (!parse_arguments(argc,
                       argv,
                       &path_to_input_rosbag
                       ))
  {
    return -1;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  constexpr double TIME_PADDING = 0.01;
  ros::Time previous_endtime = ros::Time(TIME_PADDING);
  ros::Duration duration_to_subtract = ros::Duration(0.0 - previous_endtime.toSec() - TIME_PADDING);
  bool was_within_crop = false;

  rosbag::Bag input_bag;
  try
  {
   input_bag.open(path_to_input_rosbag, rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
   std::cerr << "Error: could not open rosbag: " << path_to_input_rosbag << std::endl;
   return -1;
  }

  rosbag::Bag output_bag;
  std::string path_to_output_rosbag = path_to_input_rosbag + ".filtered";
  output_bag.open(path_to_output_rosbag, rosbag::bagmode::Write);

  rosbag::View view(input_bag);
  foreach(rosbag::MessageInstance const m, view)
  {
    const ros::Time timestamp = m.getTime();
    if (time_within_crop(timestamp))
    {
      if (!was_within_crop)
      {
        duration_to_subtract = ros::Duration(timestamp.toSec() - previous_endtime.toSec() - TIME_PADDING);
        was_within_crop = true;
      }
      const ros::Time new_timestamp = timestamp - duration_to_subtract;

      if (new_timestamp < ros::TIME_MIN)  // sanity check
      {
        std::cerr << "new timestamp < ros::TIME_MIN, skipping." << std::endl;
        continue;
      }
      output_bag.write(m.getTopic(), new_timestamp, m);
    }
    else
    {
      if (was_within_crop)
      {
        previous_endtime = timestamp - duration_to_subtract;
      }
      was_within_crop = false;
    }
  }

//  std::cout << "Wrote " << msg_count - 1 << " messages." << std::endl;

  output_bag.close();
  input_bag.close();

  return 0;
}
