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
                     int& height,
                     int& width)
{
//  std::cout << argc << std::endl;
  if(argc < 2)
  {
    std::cerr << "Not enough arguments" << std::endl;
    std::cerr << "Usage: rosrun dvs_frame_filter dvs_frame_filter path_to_bag.bag";
    return false;
  }

  *path_to_input_rosbag = std::string(argv[1]);

  if (argc == 4)
  {
    height = std::stoi(argv[2]);
    width = std::stoi(argv[3]);
  }

  return true;
}

int mode(std::vector<int> a)
{
  int previousValue = a.front();
  int mode = previousValue;
  int count = 1;
  int modeCount = 1;

  for (auto const& value: a)
  {
        if (value == previousValue)
        { // count occurrences of the current value
           ++count;
        }
        else
        { // now this is a different value
          if (count > modeCount)  // first check if count exceeds the current mode
          {
            modeCount = count;
            mode = previousValue;
          }
         count = 1; // reset count for the new number
         previousValue = value;
    }
  }

  return mode;
}

void correct_frame_size(std::string path_to_input_rosbag,
                        int& height,
                        int& width)
{
  rosbag::Bag input_bag;
  try
  {
    input_bag.open(path_to_input_rosbag, rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException e)
  {
    std::cerr << "Error: could not open rosbag: " << path_to_input_rosbag << std::endl;
    return;
  }

  rosbag::View view(input_bag);
  height = 0; // defaults
  width = 0;
  std::vector<int> height_vec;
  std::vector<int> width_vec;
  constexpr int N_SAMPLES = 100;
  int event_message_count = 0;
  int image_message_count = 0;

  // try and infer correct frame dimensions from
  // N samples of EventArray message and
  // 2N samples of Image messages.

  foreach(rosbag::MessageInstance const m, view)
  {
    if(m.getDataType() == "dvs_msgs/EventArray" && event_message_count < N_SAMPLES)
    {
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();

      height_vec.push_back(s->height);
      width_vec.push_back(s->width);
      event_message_count++;
    }

    if(m.getDataType() == "sensor_msgs/Image" && image_message_count < 2*N_SAMPLES)
    {
      sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();

      height_vec.push_back(img_msg->height);
      width_vec.push_back(img_msg->width);
      image_message_count++;
    }

    if (height_vec.size() == 3*N_SAMPLES && width_vec.size() == 3*N_SAMPLES)
    {
      break;
    }
  }

  if (height_vec.size() == 0 || width_vec.size() == 0)
  {
    input_bag.close();
    return;
  }

  sort(height_vec.begin(), height_vec.end());
  sort(width_vec.begin(), width_vec.end());

  height = mode(height_vec);
  width = mode(width_vec);

  input_bag.close();
  return;
}

int main(int argc, char* argv[])
{
  std::string path_to_input_rosbag;
  int height = 0;
  int width = 0;
  int num_bad_frames = 0;

  if (!parse_arguments(argc,
                       argv,
                       &path_to_input_rosbag,
                       height,
                       width))
  {
    return -1;
  }

  if (height == 0 || width == 0)
  {
    correct_frame_size(path_to_input_rosbag, height, width);
    std::cout << "Auto-detected height, width: " << height << ", " << width << std::endl;
  }
  else
  {
    std::cout << "Manually entered height, width: " << height << ", " << width << std::endl;
  }

  if (height == 0 || width == 0)
  {
    std::cerr << "Error: could not automatically determine correct frame size." << std::endl;
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
    if(m.getDataType() == "sensor_msgs/Image")
    {
      sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
      if (img_msg->height == height && img_msg->width == width)
      {
        output_bag.write(m.getTopic(), img_msg->header.stamp, m);
      }
      else
      {
        num_bad_frames++;
      }
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

  std::cout << "Removed " << num_bad_frames << " bad frames." << std::endl;

  output_bag.close();
  input_bag.close();

  return 0;
}
