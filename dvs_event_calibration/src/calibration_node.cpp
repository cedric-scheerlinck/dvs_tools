#include <dvs_msgs/EventArray.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <rosbag/message_instance.h>
//#include <string>
#include <ctime>

#include "dvs_event_calibration/bag_player.h"
#include "dvs_event_calibration/dvs_event_calibration.h"
#include "dvs_event_calibration/utils.h"

int main(int argc, char* argv[])
{
  // Initialize Google's flags and logging libraries.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "calibration_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_event_calibration::Calibrator calibrator(nh, nh_private);


  bool realtime = false; // bag_path.empty(); // used to determine whether to use realtime or offline mode

  if (!realtime)
  {
    std::string working_dir;
    std::string save_dir;
  
    nh_private.getParam("save_dir", save_dir);
    nh_private.getParam("working_dir", working_dir);
    if (save_dir.empty())
    {
      LOG(ERROR) << "Must specify valid save_dir!";
      return -1;
    }
    save_dir = dvs_event_calibration::utils::fullpath(working_dir, save_dir);
    if (save_dir.back() != '/')
    {
      save_dir.append("/");
    }
    const int dir_err = system((std::string("mkdir -p ") + save_dir).c_str());
    if (-1 == dir_err)
    {
        LOG(ERROR) << "Error creating save directory!";
        return -1;
    }

    std::string config_filepath;
    nh_private.getParam("config_filepath", config_filepath);
    std::ifstream infile(config_filepath);
    std::string bag_path;
	while (std::getline(infile, bag_path))
    {
      bag_path = dvs_event_calibration::utils::fullpath(working_dir, bag_path);
      std::string event_topic_name = dvs_event_calibration::utils::find_event_topic(bag_path);
  
      VLOG(1) << "Path to rosbag: " << bag_path;
      VLOG(1) << "Reading events from: " << event_topic_name;
      VLOG(1) << "Saving to: " << save_dir;
  
      // attach relevant callbacks to topics
      rpg_common_ros::BagPlayer player(bag_path);
      player.attachCallbackToTopic(event_topic_name,
          [&](const rosbag::MessageInstance& msg)
          {
            dvs_msgs::EventArray::ConstPtr events = msg.instantiate<dvs_msgs::EventArray>();
            CHECK(events);
            calibrator.eventsCallback(events);
          }
      );
  //    std::clock_t start;
  //    double duration;
  //    start = std::clock();
      player.play();
    }
    calibrator.save_calibration(save_dir);
  //    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  //    std::cout<< "printf: " << duration <<'\n';
    VLOG(1) << "...done!";
  }
  else if (realtime)
  {
    VLOG(1) << "Running in real-time mode";
    // subscriber queue size
    constexpr int EVENT_SUB_QUEUE_SIZE = 1000;
    ros::Subscriber event_sub = nh.subscribe(
        "events", EVENT_SUB_QUEUE_SIZE, &dvs_event_calibration::Calibrator::eventsCallback,
        &calibrator);
    ros::spin();
  }
  ros::shutdown();
  return 0;
}
