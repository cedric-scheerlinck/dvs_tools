#include "dvs_hot_pixel_filter/utils.h"

#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

DECLARE_bool(no_stats);

namespace dvs_hot_pixel_filter {
namespace utils {

const std::string OUTPUT_FOLDER = "./stats/";

bool parse_arguments(int argc, char* argv[],
                     std::string* path_to_input_rosbag)
{
  if(argc < 2)
  {
    std::cerr << "Error: not enough input arguments.\n"
        "Usage:\n\trosrun dvs_hot_pixel_filter hot_pixel_filter path_to_bag.bag\n\n"
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

void build_histograms(rosbag::View& view,
                      topic_mats& histograms)
{
  std::cout << "Building event count histogram(s)..." << std::endl;

  std::vector<std::string> seen_topics;
  for(const rosbag::MessageInstance& m : view)
  {
    if(m.getDataType() == "dvs_msgs/EventArray")
    {
      const std::string topic_name = m.getTopic();
      // pointer to the message
      dvs_msgs::EventArrayConstPtr s = m.instantiate<dvs_msgs::EventArray>();
      const cv::Size msg_size = cv::Size(s->width, s->height);

      cv::Mat& histogram = histograms[topic_name];

      // initialise event_count_histogram if we haven't seen the topic yet
      if ( !contains(topic_name, seen_topics) )
      {
        histogram = cv::Mat::zeros(msg_size, CV_64FC1);
        seen_topics.push_back(topic_name);
        std::cout << "added " << topic_name << " to seen_topics" << std::endl;
      }

      if (msg_size != histogram.size())
      {
        std::cerr << "Error: a new event message in " << topic_name <<
        " does not match the existing topic histogram size.\n message: " <<
        msg_size << "\t histogram: " << histogram.size() << std::endl;
        return;
      }

      for(auto e : s->events)
      {
        // accumulate events without discrimination
        histogram.at<double>(e.y, e.x)++;
      }
    }
  }

  std::cout << "...done!" << std::endl;
}

void detect_hot_pixels(const topic_mats& histograms_by_topic,
                       const double& num_std_devs,
                       const int num_hot_pixels,
                       topic_points& hot_pixels_by_topic)
{
  for(const auto& topic : histograms_by_topic)
    {
      const std::string topic_name = topic.first;
      const cv::Mat& histogram = topic.second;
      std::vector<cv::Point>& hot_pixels = hot_pixels_by_topic[topic_name];
      if (num_hot_pixels == -1)
      {
        // auto-detect hot pixels
        double threshold;
        dvs_hot_pixel_filter::utils::find_threshold(
            histogram, num_std_devs, threshold);
        dvs_hot_pixel_filter::utils::hot_pixels_by_threshold(
            histogram, threshold, hot_pixels);
      }
      else
      {
        // user-specified number of hot pixels
        dvs_hot_pixel_filter::utils::hot_pixels_by_ranking(
            histogram, num_hot_pixels, hot_pixels);
      }
    }
}

void hot_pixels_by_threshold(const cv::Mat& histogram,
                             const double& threshold,
                             std::vector<cv::Point>& hot_pixels)
{
  for (int y = 0; y < histogram.rows; y++)
  {
    for (int x = 0; x < histogram.cols; x++)
    {
      if (histogram.at<double>(y, x) > threshold)
      {
        hot_pixels.push_back(cv::Point(x, y));
      }
    }
  }
}
void hot_pixels_by_ranking(const cv::Mat& histogram,
                           const double& num_hot_pixels,
                           std::vector<cv::Point>& hot_pixels)
{
  cv::Mat local_hist;
  histogram.copyTo(local_hist);

  for (int i = 0; i < num_hot_pixels; i++)
  {
    double max;
    cv::Point maxLoc;
    cv::minMaxLoc(local_hist, nullptr, &max, nullptr, &maxLoc);

    hot_pixels.push_back(maxLoc);
    local_hist.at<double>(maxLoc) = 0;
  }
}

void find_threshold(const cv::Mat& histogram,
                    const double num_std_devs,
                    double& threshold)
{
  cv::Scalar mean_Scalar, stdDev_Scalar;
  cv::meanStdDev(histogram, mean_Scalar, stdDev_Scalar, histogram > 0);

  const double mean = mean_Scalar[0];
  const double stdDev = stdDev_Scalar[0];
  threshold = mean + num_std_devs*stdDev;
}

void write_all_msgs(rosbag::View& view,
                    topic_points& hot_pixels_by_topic,
                    rosbag::Bag& output_bag)
{
  constexpr int log_every_n_messages = 10000;
  const uint32_t num_messages = view.size();
  uint32_t message_index = 0;
  std::cout << "Writing..." << std::endl;
  // write the new rosbag without hot pixels by iterating over all messages
  for(rosbag::MessageInstance const m : view)
  {
    write_msg(
        m, hot_pixels_by_topic, output_bag);
    if(message_index++ % log_every_n_messages == 0)
    {
      std::cout << "Message: " << message_index << " / " << num_messages << std::endl;
    }
  }

  std::cout << "Message: " << num_messages << " / " << num_messages << std::endl;
  std::cout << "...done!" << std::endl;
}

void write_event_msg(const std::string topic_name,
                     const dvs_msgs::EventArrayConstPtr event_array_ptr,
                     const std::vector<cv::Point>& hot_pixels,
                     rosbag::Bag& output_bag)
{
  std::vector<dvs_msgs::Event> events;
  for(auto e : event_array_ptr->events)
  {
    if (!contains(cv::Point(e.x, e.y), hot_pixels))
    {
      events.push_back(e);
    }
  }

  if (events.size() > 0)
  {
  // Write new event array message to output rosbag

  dvs_msgs::EventArray event_array_msg;
  event_array_msg.events = events;
  event_array_msg.width = event_array_ptr->width;
  event_array_msg.height = event_array_ptr->height;
  event_array_msg.header.stamp = events.back().ts;

  output_bag.write(topic_name, event_array_msg.header.stamp, event_array_msg);
  }
}

void write_msg(const rosbag::MessageInstance& m,
               topic_points& hot_pixels_topic,
               rosbag::Bag& output_bag)
{
  if(m.getDataType() == "dvs_msgs/EventArray")
  {
    const std::string topic_name = m.getTopic();
    std::vector<cv::Point>& hot_pixels = hot_pixels_topic[topic_name];
    dvs_msgs::EventArrayConstPtr event_array_ptr = m.instantiate<dvs_msgs::EventArray>();
    write_event_msg(topic_name, event_array_ptr, hot_pixels, output_bag);

  }
  else if(m.getDataType() == "sensor_msgs/Image")
  {
    sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
    output_bag.write(m.getTopic(), img_msg->header.stamp, m);
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

std::string usable_filename(const std::string filename_in)
{
  std::string filename = filename_in;
  std::replace( filename.begin(), filename.end(), '/', '_'); // replace all '/' to '_'
  std::replace( filename.begin(), filename.end(), '\\', '_'); // replace all '\' to '_'
  return filename;
}

void write_hot_pixels(const std::string filename,
                      const std::vector<cv::Point>& hot_pixels)
{
  std::ofstream hot_pixels_file;
  hot_pixels_file.open(filename);
  // the important part
  for (const auto& point : hot_pixels)
  {
    hot_pixels_file << point.x << ", " << point.y << "\n";
  }
  hot_pixels_file.close();
}

void save_stats(const std::string bag_name,
                 const std::string topic_name,
                 const cv::Mat& histogram,
                 const std::vector<cv::Point>& hot_pixels,
                 const bool one_topic)
{
  cv::Mat histogram_after;
  histogram.copyTo(histogram_after);
  for (auto point : hot_pixels)
  {
    histogram_after.at<double>(point) = 0;
  }

  const double num_events = cv::sum(histogram)[0];
  const double num_events_after = cv::sum(histogram_after)[0];
  const double percent_events_discarded = (1 - num_events_after/num_events)*100;

  std::cout << std::setprecision(4) << topic_name << "\t" << num_events <<
      "\t" << hot_pixels.size() << "\t\t0\t(before)" << std::endl;

  std::cout << std::setprecision(4) << topic_name << "\t" << num_events_after <<
      "\t0\t\t" << percent_events_discarded << "\t(after)" << std::endl;

  if (!FLAGS_no_stats)
  {
    // save images
    std::string dstDir = OUTPUT_FOLDER + bag_name + "/";
    if (!one_topic)
    {
      dstDir += usable_filename(topic_name) + "/";
    }
    boost::filesystem::create_directories(dstDir); // create if needed
    std::string fname_b = dstDir + "hist_before.png";
    std::string fname_a = dstDir + "hist_after.png";
    std::string fname_hp = dstDir + "hot_pixels.txt";

    write_histogram_image(fname_b, histogram);
    write_histogram_image(fname_a, histogram_after, hot_pixels);
    write_hot_pixels(fname_hp, hot_pixels);
  }
}

}  // namespace utils
}  // namespace dvs_hot_pixel_filter
