//
// Created by Mel on 11/16/2021.
//

#ifndef TEST_GLOVE_CAPTURE_POINTCLOUD_H
#define TEST_GLOVE_CAPTURE_POINTCLOUD_H

#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <map>
#include <functional>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h> //and the other usuals

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>

#include "json.hpp"

// #include <windows.h>

using namespace std;

class CapturePointCloud {
    // void cloudViewer(void);

    // Global Variables
private:
    string cloudFile; // .pcd file name
    string spheresFile;
    string prevCloudFile; // .pcd file name (Old cloud)
    int fileCount = 1; // Index for incremental file name

    std::ifstream *infile;
    nlohmann::json *config;
    double h_lt, h_gt, v_lt, v_gt, z_lt, z_gt;

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    rs2::pipeline_profile selection;
    rs2::device selected_device;

    //====================
    // Object Declaration
    //====================
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr dst;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb_only;
    pcl::PointCloud<pcl::Normal>::Ptr normal_only;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final;

public:
    CapturePointCloud();
    std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Conversion(const rs2::points &points, const rs2::video_frame &color);

    void Load_PCDFile(std::string openFileName);
    int getFrame();

    //bool userInput(void);

};

#endif //TEST_GLOVE_CAPTURE_POINTCLOUD_H
