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
                     std::string* path_to_input_rosbag,
                     int& num_msgs,
                     int& num_skip)
//                     double& duration)
{
//  std::cout << argc << std::endl;
  if(argc < 3)
  {
    std::cerr << "Not enough arguments" << std::endl;
    std::cerr << "Usage: rosrun dvs_crop_bag dvs_crop_bag path_to_bag.bag num_msgs";
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);

//  duration = std::stod(argv[2]);

  num_msgs = std::stoi(argv[2]);

  if (argc == 4)
  {
    num_skip = std::stoi(argv[3]);
  }

  return true;
}

int main(int argc, char* argv[])
{
  std::string path_to_input_rosbag;
  int num_msgs;
  int num_skip = 0;
  int msg_count = 0;
  int skip_count = 0;

  if (!parse_arguments(argc,
                       argv,
                       &path_to_input_rosbag,
                       num_msgs,
                       num_skip))
  {
    return -1;
  }

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
    if (skip_count++ < num_skip)
    {
      continue;
    }

    if (msg_count++ >= num_msgs)
    {
      break;
    }

    if(m.getDataType() == "sensor_msgs/Image")
    {
      sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
      output_bag.write(m.getTopic(), img_msg->header.stamp, m);
    }
    else if(m.getDataType() == "dvs_msgs/EventArray")
    {
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();
      ros::Time timestamp = s->header.stamp;
      if (timestamp < ros::TIME_MIN)
      {
        timestamp = s->events.back().ts;
      }
      output_bag.write(m.getTopic(), timestamp, m);
    }
    else if(m.getDataType() == "sensor_msgs/Imu")
    {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      output_bag.write(m.getTopic(), imu_msg->header.stamp, m);
    }
    else
    {
      output_bag.write(m.getTopic(), m.getTime(), m);
    }
  }

  std::cout << "Wrote " << msg_count - 1 << " messages." << std::endl;

  output_bag.close();
  input_bag.close();

  return 0;
}
