#ifndef CALIB_IO_H
#define CALIB_IO_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

/*!
 * \brief The messageIO class, includes rosbag interface
 * and message type transform.
 */
class messageIO {

public:

  messageIO();

  ~messageIO(){

  }

  /*!
   * \brief The laserScanData struct. Used for storing data
   * read from LiDAR.
   */
  struct laserScanData {
    ros::Time timestamp;
    float time_increment;
    float scan_time;
    float max_range;
    float min_range;
    float angle_increment;
    float min_angle;
    float max_angle;
    std::vector<float> ranges;
  };

  /*!
   * \brief readDataFromBag. Member function used for
   * reading data from a ROS bag.
   * \param bag_name. Input as a string.
   * \param laser_topic_name. Input as a string.
   * \param odom_topic_name. Input as a string.
   * \param odom_data. Vector of odometer data
   * transformed from ROS message.
   * \param laser_data. Vector of laser data
   * transformed from ROS message.
   */
  bool readDataFromBag(const std::string &bag_name, const std::string &laser_topic_name,
                    std::vector<laserScanData> &laser_data,
                    std::vector<sensor_msgs::LaserScan>& laser_scans);

};

#endif
