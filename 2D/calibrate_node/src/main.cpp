#include <ros/ros.h>
#include "calibrate.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_calibrate");
  ros::NodeHandle n;

  LaserCalibrate lc;
  lc.Start();

  ros::spin();

  return 0;
}
