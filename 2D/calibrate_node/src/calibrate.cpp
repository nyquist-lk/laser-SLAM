#include "calibrate.h"

LaserCalibrate::LaserCalibrate():
m_status(FREE)
{
    m_params.x = 1.0;
    m_params.y = 1.0;
    m_params.yaw = 0.0;
}

LaserCalibrate::~LaserCalibrate()
{

}


void LaserCalibrate::Start()
{
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("line_bag_file", m_line_bag_file, "line.bag");
    private_nh.param<std::string>("circle_bag_file", m_circle_bag_file, "circle.bag");
    private_nh.param<std::string>("laser_topic", m_laser_topic, "/scan");

    ROS_INFO("line_bag_file: %s", m_line_bag_file.c_str());
    ROS_INFO("circle_bag_file: %s", m_circle_bag_file.c_str());

    m_odom_pub = private_nh.advertise<nav_msgs::Odometry>("/laser_odom", 50);

    m_calibrate_tid = std::thread(&LaserCalibrate::ThreadCalibrate, this);
}

void LaserCalibrate::ThreadCalibrate()
{
    ros::Rate loop_rate(2);
    m_status = CALIBRATE;
    while (ros::ok())
    {
        switch (m_status)
        {
        case FREE:
            break;
        case CALIBRATE:
            CalibrateProcess();
            break;

        case FAIL:
            break;
        
        case SUCCESS:
            m_status = FREE;
            break;
        
        default:
            break;
        }

        loop_rate.sleep();
    }
}

void LaserCalibrate::Downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
                        pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
    // Uniform sampling object.
    pcl::UniformSampling<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud_in);
    filter.setRadiusSearch(0.01f);
    // We need an additional object to store the indices of surviving points.
    pcl::PointCloud<int> keypointIndices;

    filter.compute(keypointIndices);
    pcl::copyPointCloud(*cloud_in, keypointIndices.points, cloud_out);
}

void LaserCalibrate::PubOdomScan(std::vector<cScanMatch::csm_odom>& scan_odom,
                            std::vector<sensor_msgs::LaserScan> laser_scans,
                            std::vector<cScanMatch::csm_results> match_results)
{
    ros::Rate loop_rate(30);
    for (size_t i = 0; i < scan_odom.size() / 2; i++)
    {
        //发布odometry topic
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = scan_odom[i].x;
        odom.pose.pose.position.y = scan_odom[i].y;
        odom.pose.pose.position.z = 0.0;

        // ROS_INFO("theta: %lf, dt_theta: %lf", scan_odom[i].theta, match_results[i].scan_match_results[2]);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(scan_odom[i].theta);
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "laser_link";
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0;

        m_odom_pub.publish(odom);

        loop_rate.sleep();
    }
}

bool LaserCalibrate::GetCalibrateData(std::vector<SolverData>& datas, 
                                        std::string bag_file)
{
    std::vector<messageIO::laserScanData> laser_datas(0);
    std::vector<sensor_msgs::LaserScan> laser_scans(0);
    if (!m_dataIO.readDataFromBag(bag_file, m_laser_topic, laser_datas, laser_scans))
    {
        ROS_INFO("readDataFromBag error!");
        return false;
    }

    std::vector<LDP> ldp(0);
    cScanMatch cScan;
    cScan.scanToLDP(laser_datas, ldp);

    std::vector<cScanMatch::csm_odom> scan_odom(0);
    std::vector<cScanMatch::csm_results> match_results(0);
    cScan.match(laser_datas, ldp, scan_odom, match_results);//获取激光雷达的最终数据

    PubOdomScan(scan_odom, laser_scans, match_results);

    for (size_t i = 1; i < scan_odom.size(); i++)
    {
        SolverData d;
        d.x = scan_odom[i].x;
        d.y = scan_odom[i].y;
        d.th = scan_odom[i].theta;
        datas.push_back(d);
    }

    // // 生成结果点云
    // // 转换为pcl点云
    ROS_INFO("to pcl pointcloud start...");
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
    laser_geometry::LaserProjection projector_;
    clouds.resize(laser_scans.size());
	for(int i = 0; i < laser_scans.size(); i++)
	{
		sensor_msgs::PointCloud2 ros_cloud;
		projector_.projectLaser(laser_scans[i], ros_cloud);
		pcl::fromROSMsg(ros_cloud, clouds[i]);
	}
    ROS_INFO("to pcl pointcloud success!");

    ROS_INFO("scans size: %d, clouds size: %d", laser_scans.size(), clouds.size());
    assert(laser_scans.size() == clouds.size());

    // 转换点云，并生成结果点云
    ROS_INFO("merge result pointcloud start...");
    // ROS_INFO("first index: %d", scan_odom[0].index);
    pcl::PointCloud<pcl::PointXYZ> result_cloud = clouds[scan_odom[0].index];
    pcl::PointCloud<pcl::PointXYZ>::Ptr transfromed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    int step = 5;
    for (int i = 1; i < scan_odom.size();)
    {
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        transform_2.translation() << scan_odom[i].x, scan_odom[i].y, 0.0;
        transform_2.rotate(Eigen::AngleAxisf(scan_odom[i].theta, Eigen::Vector3f::UnitZ()));

        // ROS_INFO("index: %d", scan_odom[i].index);
        // 执行变换，并将结果保存在新创建的‎‎ transformed_cloud ‎‎中
        if (i >= 2)
        {
            transfromed_cloud->clear();
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ> cloud = clouds[scan_odom[i].index];
        Downsample(cloud.makeShared(), *filter_cloud);
        pcl::transformPointCloud(*filter_cloud, *transfromed_cloud, transform_2);

        result_cloud = result_cloud + *transfromed_cloud;

        // ROS_INFO("result_cloud size: %d", result_cloud.size());

        i += step;
    }
    ROS_INFO("merge result pointcloud success!");

    std::string filename = "tmp";
    if (bag_file.find("line") != string::npos)
    {
        filename = "line";
    }
    if (bag_file.find("circle") != string::npos)
    {
        filename = "circle";
    }
    std::string path = "/robot_data/calibration/" + filename + ".pcd";
    pcl::io::savePCDFileBinary(path.c_str(), result_cloud);
    ROS_INFO("save pointcloud %s success!", path.c_str());

    // empty vector
    {
        laser_datas.clear();
        std::vector<messageIO::laserScanData>().swap(laser_datas);
        laser_scans.clear();
        std::vector<sensor_msgs::LaserScan>().swap(laser_scans);
        clouds.clear();
        std::vector<pcl::PointCloud<pcl::PointXYZ>>().swap(clouds);
        result_cloud.clear();
    }

    return true;
}

bool LaserCalibrate::LineCalibrate()
{
    ROS_INFO("line Calirbate start ...");
    std::vector<SolverData> datas(0);
    if (!GetCalibrateData(datas, m_line_bag_file))
    {
        ROS_INFO("LineCalibrate error!");
        m_status = FAIL;
        return false;
    }

    double angle_mean = 0.0;
    for (size_t i = 0; i < datas.size(); i++)
    {
        angle_mean += datas[i].th;
    }
    angle_mean /= double(datas.size());

    std::vector<double> params(0);
    GSSlover gssolver;
    if (!gssolver.GetOptimzeParams(datas, "line", params))
    {
        ROS_INFO("GetOptimzeParams line error!");
        m_status = FAIL;
        return false;
    }

    ROS_INFO("ceres optimze: %lf, %lf",
                params[0], params[1]);
    
    double ceres_angle = std::atan(params[0]);
    ROS_INFO("ceres theta: %lf, angle_mean: %lf, delta_angle: %lf",
                ceres_angle, angle_mean, (ceres_angle - angle_mean));

    ROS_INFO("line Calirbate success!");

    return true;
}

bool LaserCalibrate::CircleCalibrate()
{
    ROS_INFO("circle Calirbate start ...");
    std::vector<SolverData> datas(0);
    if (!GetCalibrateData(datas, m_circle_bag_file))
    {
        ROS_INFO("CircleCalibrate error!");
        m_status = FAIL;
        return false;
    }

    std::vector<double> params(0);
    GSSlover gssolver;
    if (!gssolver.GetOptimzeParams(datas, "circle", params))
    {
        ROS_INFO("GetOptimzeParams circle error!");
        m_status = FAIL;
        return false;
    }

    double R = std::sqrt(params[2]);
    ROS_INFO("x: %lf, y: %lf, R: %lf", params[0], params[1], R);

    ROS_INFO("circle Calirbate success!");

    return true;
}

bool LaserCalibrate::CalibrateProcess()
{
    ROS_INFO("Calirbate start ...");
    // 激光与车体之间角度标定
    if (!LineCalibrate())
    {
        ROS_INFO("line Calirbate failed!");
        return false;
    }

    // 激光与车体之间xy值标定
    // if (!CircleCalibrate())
    // {
    //     ROS_INFO("circle Calirbate failed!");
    //     return false;
    // }

    ROS_INFO("Calirbate success!");

    m_status = SUCCESS;
    return true;
}