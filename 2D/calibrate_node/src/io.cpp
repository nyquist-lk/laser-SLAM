#include "io.h"

messageIO::messageIO()
{
  // ROS_INFO("Initing IO...");
}

bool messageIO::readDataFromBag(const std::string &bag_name, 
                            const std::string &laser_topic_name,
                            std::vector<laserScanData> &laser_data,
                            std::vector<sensor_msgs::LaserScan>& laser_scans)
{
  rosbag::Bag bag;
  ROS_INFO("Opening bag...");

  try{
    bag.open(bag_name, rosbag::bagmode::Read);
  }
  catch (std::exception& e) {
    ROS_INFO("ERROR!");
    return false;
  }

  ROS_INFO("DONE!");
  ROS_INFO("Quering topics bag...");

  std::vector<std::string> topics;
  topics.push_back(laser_topic_name);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // std::cout << colouredString("Reading bag data...", YELLOW, REGULAR) << std::endl;

  for (rosbag::MessageInstance const m: view) {
    sensor_msgs::LaserScanConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
    if (scan != NULL) {
      laserScanData tmp;
      tmp.ranges = scan->ranges;
      tmp.scan_time = scan->scan_time;
      tmp.time_increment = scan->time_increment;
      tmp.timestamp = scan->header.stamp;
      tmp.angle_increment = scan->angle_increment;
      tmp.max_angle = scan->angle_max;
      tmp.min_angle = scan->angle_min;
      tmp.max_range = scan->range_max;
      tmp.min_range = scan->range_min;
      laser_data.push_back(tmp);

      laser_scans.push_back(*scan);
    }
  }

  ROS_INFO("Data reading finished!");

  if (laser_data.size() <= 1)
  {
    ROS_INFO("Data size: %d", laser_data.size());
    return false;
  }

  return true;

}
