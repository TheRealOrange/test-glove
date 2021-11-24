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
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h> //and the other usuals

#include "json.hpp"

// #include <windows.h>

using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;

// void cloudViewer(void);

// Global Variables
string cloudFile; // .pcd file name
string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color);
void Load_PCDFile(void);
bool userInput(void);

#endif //TEST_GLOVE_CAPTURE_POINTCLOUD_H
