#include <ros/ros.h>
#include <algorithm>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_align");
    ros::NodeHandle nh, nh_private("~");

    //~ STEP 1.0: Init loader, lidar, and odom(IMU)
    Loader loader(Loader::getConfig(&nh_private));      //~ default: load all data. (`use_n_data = maximum of the platform.)
    Lidar lidar;
    Odom odom;

    //~ STEP 1.1: Load all lidar data (to lidar.scan_) from Bag.
    std::string input_bag_path;
    ROS_INFO("Loading Pointcloud Data...");
    if (!nh_private.getParam("input_bag_path", input_bag_path)){
        ROS_FATAL("Could not find input_bag_path parameter, exiting");
        exit(EXIT_FAILURE);
    }
    else if (!loader.loadPointcloudFromROSBag(input_bag_path, Scan::getConfig(&nh_private), &lidar)){
        ROS_FATAL("Error loading pointclouds from ROS bag.");
        exit(0);
    }

    //~ STEP 1.2 Load odom(IMU) data (all transform T) from Bag.
    bool transforms_from_csv;
    nh_private.param("transforms_from_csv", transforms_from_csv, false);
    std::string input_csv_path;
    ROS_INFO("Loading Transformation Data...");
    if (transforms_from_csv){
        if (!nh_private.getParam("input_csv_path", input_csv_path)){
            ROS_FATAL("Could not find input_csv_path parameter, exiting");
            exit(EXIT_FAILURE);
        }
        else if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom)){
            ROS_FATAL("Error loading transforms from CSV.");
            exit(0);
        }
    }
    else if (!loader.loadTformFromROSBag(input_bag_path, &odom)){       //~ default: load from ROS bag.
        ROS_FATAL("Error loading transforms from ROS bag.");
        exit(0);
    }

    if (lidar.getNumberOfScans() == 0){
        ROS_FATAL("No data loaded, exiting");
        exit(0);
    }
    ROS_INFO("Interpolating Transformation Data...");

    //~ STEP 2. Calculating.
    lidar.setOdomOdomTransforms(odom);
    Aligner aligner(Aligner::getConfig(&nh_private));
    aligner.lidarOdomTransform(&lidar, &odom);

    return 0;
}
