#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <thread>
#include <mutex>

#include "io.h"
#include "csm/laser_data.h"
#include "scan_match.h"
#include "solver.h"

using namespace std;

struct Parms
{
    double x;
    double y;
    double yaw;
};

enum CalibrateState {FREE, CALIBRATE, FAIL, SUCCESS};

class LaserCalibrate
{
public:
    LaserCalibrate();

    ~LaserCalibrate();

    void Start();

private:
    ros::NodeHandle m_nh;

    void ThreadCalibrate();

    void Downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
                    pcl::PointCloud<pcl::PointXYZ>& cloud_out);
    bool GetCalibrateData(std::vector<SolverData>& datas, 
                            std::string bag_file);

    bool LineCalibrate();
    bool CircleCalibrate();
    bool CalibrateProcess();

    void PubOdomScan(std::vector<cScanMatch::csm_odom>& scan_odom,
                std::vector<sensor_msgs::LaserScan> laser_scans,
                std::vector<cScanMatch::csm_results> match_results);

private:
    std::string m_line_bag_file, m_circle_bag_file;
    std::string m_laser_topic;
    messageIO m_dataIO;

    CalibrateState m_status;

    std::mutex m_data_mtx;
    std::thread m_calibrate_tid;

    ros::Publisher m_odom_pub;

    Parms m_params;
};