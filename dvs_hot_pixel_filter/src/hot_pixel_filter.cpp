#include "dvs_hot_pixel_filter/utils.h"

#include <gflags/gflags.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

constexpr double NUM_STD_DEVS = 5; // default

DEFINE_double(n_std, NUM_STD_DEVS, "Number of standard deviations for hot pixel threshold");
DEFINE_int32(n_hot_pix, -1, "Number of hot pixels to be removed");
DEFINE_bool(no_stats, false, "Do not save statistics to disk");

int main(int argc, char* argv[])
{ 
  google::ParseCommandLineFlags(&argc, &argv, true);

  // parse input arguments and setup input and output rosbags
  std::string path_to_input_rosbag;

  if(!dvs_hot_pixel_filter::utils::parse_arguments(argc, argv, &path_to_input_rosbag))
  {
    return -1;
  }

  if (FLAGS_n_hot_pix == -1)
  {
    std::cout << "Number of hot pixels will be determined automatically\n"
        "by thresholding event_count > " << FLAGS_n_std << " STD (standard deviations)" << std::endl;
  } else
  {
    std::cout << "The " << FLAGS_n_hot_pix << " hottest pixel(s) will be removed" << std::endl;
  }

  std::string bag_name = dvs_hot_pixel_filter::utils::extract_bag_name(
      path_to_input_rosbag);

  std::string path_to_output_rosbag = path_to_input_rosbag + ".filtered";

  rosbag::Bag input_bag;
  try
  {
    input_bag.open(path_to_input_rosbag, rosbag::bagmode::Read);
  }
  catch(rosbag::BagIOException& e)
  {
    std::cerr << "Error: could not open rosbag: " << path_to_input_rosbag << std::endl;
    return -1;
  }

  rosbag::Bag output_bag;
  output_bag.open(path_to_output_rosbag, rosbag::bagmode::Write);

  rosbag::View view(input_bag);

  // initialise variables and start computation
  dvs_hot_pixel_filter::utils::topic_mats histograms_by_topic;
  dvs_hot_pixel_filter::utils::topic_points hot_pixels_by_topic;

  dvs_hot_pixel_filter::utils::build_histograms(
      view, histograms_by_topic);

  dvs_hot_pixel_filter::utils::detect_hot_pixels(
      histograms_by_topic, FLAGS_n_std, FLAGS_n_hot_pix, hot_pixels_by_topic);

  dvs_hot_pixel_filter::utils::write_all_msgs(
      view, hot_pixels_by_topic, output_bag);

  std::cout << "Computing stats..." << std::endl;

  output_bag.close();
  input_bag.close();

  // write statistics
  const bool one_topic = histograms_by_topic.size() == 1;
  std::cout << "Topic\t\t# Events\t#Hot pixels\t% Events discarded" << std::endl;
  for(auto topic : histograms_by_topic)
  {
    const std::string topic_name = topic.first;
    cv::Mat& histogram = topic.second;
    std::vector<cv::Point>& hot_pixels = hot_pixels_by_topic[topic_name];
    dvs_hot_pixel_filter::utils::save_stats(
        bag_name, topic_name, histogram, hot_pixels, one_topic);
  }

  return 0;
}
