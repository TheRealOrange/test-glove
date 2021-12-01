//
// Created by Mel on 11/16/2021.
//
#include <iostream>
#include <librealsense2/rs.hpp>
#include "../includes/capture_pointcloud.h"

std::tuple<int, int, int> CapturePointCloud::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CapturePointCloud::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width  = static_cast<uint32_t>( sp.width()  );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<0>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<2>(RGB_Color); // Reference tuple<0>

    }

    return cloud; // PCL RGB Point Cloud generated
}
/*

int main() try
{
    //======================
    // Variable Declaration
    //======================
    bool captureLoop = true; // Loop control for generating point clouds


    // Begin Stream with default configs

    // Loop and take frame captures upon user input
    while(captureLoop == true) {

        // Set loop flag based on user input
        captureLoop = userInput();
        if (captureLoop == false) { break; }



        // Capture a single frame and obtain depth + RGB values from it
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();

        // Map Color texture to each point
        pc.map_to(RGB);

        // Generate Point Cloud
        auto points = pc.calculate(depth);

        // Convert generated Point Cloud to PCL Formatting
        cloud_pointer cloud = PCL_Conversion(points, RGB);

        //========================================
        // Compute Normals
        //========================================

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod(tree);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(0.03);

        // Compute the features
        ne.compute (*cloud_normals);

        // Initialization part
        dst->width = cloud->width;
        dst->height = cloud->height;
        dst->is_dense = true;
        dst->points.resize(dst->width * dst->height);

        // Assignment part
        for (int i = 0; i < cloud_normals->points.size(); i++)
        {
            dst->points[i].x = cloud->points[i].x;
            dst->points[i].y = cloud->points[i].y;
            dst->points[i].z = cloud->points[i].z;

            dst->points[i].r = cloud->points[i].r;
            dst->points[i].g = cloud->points[i].g;
            dst->points[i].b = cloud->points[i].b;

            // cloud_normals -> Which you have already have; generated using pcl example code

            dst->points[i].curvature = cloud_normals->points[i].curvature;

            dst->points[i].normal_x = cloud_normals->points[i].normal_x;
            dst->points[i].normal_y = cloud_normals->points[i].normal_y;
            dst->points[i].normal_z = cloud_normals->points[i].normal_z;
        }

        //========================================
        // Filter PointCloud (PassThrough Method)
        //========================================
        pcl::PassThrough<pcl::PointXYZRGBNormal> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud (dst);           // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits (z_gt, z_lt);      // Set accepted interval values
        Cloud_Filter.filter (*newCloud);              // Filtered Cloud Outputted
//
//        pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter1; // Create the filtering object
//        Cloud_Filter.setInputCloud (newCloud);           // Input generated cloud to filter
//        Cloud_Filter.setFilterFieldName ("r");        // Set field name to Z-coordinate
//        Cloud_Filter.setFilterLimits (100, 255.0);      // Set accepted interval values
//        Cloud_Filter.filter (*newerCloud);              // Filtered Cloud Outputted

        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> color_filter;

        pcl::PackedHSIComparison<pcl::PointXYZRGBNormal>::Ptr
                hue_lt(new pcl::PackedHSIComparison<pcl::PointXYZRGBNormal>("h", pcl::ComparisonOps::LT, h_lt));
        pcl::PackedHSIComparison<pcl::PointXYZRGBNormal>::Ptr
                hue_gt(new pcl::PackedHSIComparison<pcl::PointXYZRGBNormal>("h", pcl::ComparisonOps::GT, h_gt));
//        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
//                sat_lt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("s", pcl::ComparisonOps::LT, s_lt));
//        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
//                sat_gt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("s", pcl::ComparisonOps::GT, s_gt));
//        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
//                val_lt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("i", pcl::ComparisonOps::LT, v_lt));
//        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
//                val_gt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("i", pcl::ComparisonOps::GT, v_gt));
        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        color_cond->addComparison (hue_lt);
        color_cond->addComparison (hue_gt);
//        color_cond->addComparison (sat_lt);
//        color_cond->addComparison (sat_gt);
//        color_cond->addComparison (val_lt);
//        color_cond->addComparison (val_gt);

        // Build the filter
        color_filter.setInputCloud(newCloud);
        color_filter.setCondition (color_cond);
        color_filter.filter(*newerCloud);

        // created RandomSampleConsensus object and compute the appropriated model
        std::vector<int> inliers;
        pcl::copyPointCloud(*newerCloud, *xyzrgb_only);
        pcl::copyPointCloud(*newerCloud, *normal_only);
        pcl::SampleConsensusModelNormalSphere<pcl::PointXYZRGB, pcl::Normal>::Ptr
            model_sphere(new pcl::SampleConsensusModelNormalSphere<pcl::PointXYZRGB, pcl::Normal> (xyzrgb_only));
        model_sphere->setInputNormals(normal_only);
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model_sphere, 0.03);
        sac.computeModel();
        sac.getInliers(inliers);
        pcl::copyPointCloud(*xyzrgb_only, inliers, *final);

        // pointcloud source cloud
        // pointcloud output newCloud

        Eigen::Matrix4f flippy_trans = Eigen::Matrix4f::Identity();
        flippy_trans (1,1) = -1; // (row, col)
        // std::cout << flippy_trans << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr newererCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
        pcl::transformPointCloud( *final, *spheres, flippy_trans);
        pcl::transformPointCloud( *newerCloud, *newererCloud, flippy_trans);

        cloudFile = "Captured_Frame" + to_string(i) + ".pcd";
        std::string spheresFile = "Sphere" + to_string(i) + ".pcd";

        //==============================
        // Write PC to .pcd File Format
        //==============================
        // Take Cloud Data and write to .PCD File Format
        cout << "Generating PCD Point Cloud File... " << endl;
        pcl::io::savePCDFileASCII(cloudFile, *spheres); // Input cloud to be saved to .pcd
        pcl::io::savePCDFileASCII(cloudFile, *newererCloud); // Input cloud to be saved to .pcd
        cout << cloudFile << " successfully generated. " << endl;

        //Load generated PCD file for viewing
        Load_PCDFile(cloudFile);
        Load_PCDFile(spheresFile);
        i++; // Increment File Name
    }//End-while


    cout << "Exiting Program... " << endl;
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
*/

void CapturePointCloud::Load_PCDFile(std::string openFileName) {

    // Generate object to store cloud in .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile (openFileName, *cloudView); // Load .pcd File

    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Captured Frame"
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Captured Frame"));

    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0);
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudView, "Cloud");
    // viewer->addPointCloud(cloudView, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // Viewer Properties
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    cout << endl;
    cout << "Press [Q] in viewer to continue. " << endl;

    viewer->spin(); // Allow user to rotate point cloud and view it

    // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.


}

CapturePointCloud::CapturePointCloud() {
    infile = new std::ifstream("config.json");
    config = new nlohmann::json();
    *infile >> *config;
    // read a JSON file
    h_gt = (*config)["h_gt"];
    h_lt = (*config)["h_lt"];
    v_gt = (*config)["v_gt"];
    v_lt = (*config)["v_lt"];
    z_gt = (*config)["z_gt"];
    z_lt = (*config)["z_lt"];

    //======================
    // Stream configuration with parameters resolved internally. See enable_stream() overloads for extended usage
    //======================
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED);
    cfg.enable_stream(RS2_STREAM_DEPTH);

    cloud_normals = static_cast<shared_ptr<pcl::PointCloud<pcl::Normal>>>(new pcl::PointCloud<pcl::Normal>());
    dst = static_cast<shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>>>(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    newCloud = static_cast<shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>(new pcl::PointCloud<pcl::PointXYZRGB>());
    filteredCloud = static_cast<shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>(new pcl::PointCloud<pcl::PointXYZRGB>());

    xyzrgb_only = static_cast<shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>(new pcl::PointCloud<pcl::PointXYZRGB>());
    normal_only = static_cast<shared_ptr<pcl::PointCloud<pcl::Normal>>>(new pcl::PointCloud<pcl::Normal>());
    final = static_cast<shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>(new pcl::PointCloud<pcl::PointXYZRGB>());

    selection = pipe.start(cfg);

    selected_device = selection.get_device();
    auto sensor = selected_device.first<rs2::depth_sensor>();

    if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        pipe.wait_for_frames();
    }

    if (sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
        sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
    }

    // Wait for frames from the camera to settle
    for (int i = 0; i < 30; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }

    cloudFile = "Captured_Frame" + to_string(0) + ".pcd";
    spheresFile = "Sphere" + to_string(0) + ".pcd";
}

int CapturePointCloud::getFrame() {
    try {
        // Capture a single frame and obtain depth + RGB values from it
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto RGB = frames.get_color_frame();

        // Map Color texture to each point
        pc.map_to(RGB);

        // Generate Point Cloud
        auto points = pc.calculate(depth);

        // Convert generated Point Cloud to PCL Formatting
        cloud = PCL_Conversion(points, RGB);


        //========================================
        // Filter PointCloud (PassThrough Method)
        //========================================
        pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits (z_gt, z_lt);      // Set accepted interval values
        Cloud_Filter.filter (*newCloud);              // Filtered Cloud Outputted

        pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
                hue_lt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("h", pcl::ComparisonOps::LT, h_lt));
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
                hue_gt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("h", pcl::ComparisonOps::GT, h_gt));
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
                val_lt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("i", pcl::ComparisonOps::LT, v_lt));
        pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
                val_gt(new pcl::PackedHSIComparison<pcl::PointXYZRGB>("i", pcl::ComparisonOps::GT, v_gt));
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        color_cond->addComparison (hue_lt);
        color_cond->addComparison (hue_gt);
        color_cond->addComparison (val_lt);
        color_cond->addComparison (val_gt);

        // Build the filter
        color_filter.setInputCloud(newCloud);
        color_filter.setCondition (color_cond);
        color_filter.filter(*filteredCloud);

        //========================================
        // Compute Normals
        //========================================

        std::cout << "\nEstimating Normals" << std::endl;
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(filteredCloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod(tree);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(0.03);

        // Compute the features
        ne.compute (*cloud_normals);
        std::cout << "done" << std::endl;

        // Initialization part
        dst->width = cloud->width;
        dst->height = cloud->height;
        dst->is_dense = true;
        dst->points.resize(dst->width * dst->height);

        // Assignment part
        for (int i = 0; i < cloud_normals->points.size(); i++)
        {
            dst->points[i].x = filteredCloud->points[i].x;
            dst->points[i].y = filteredCloud->points[i].y;
            dst->points[i].z = filteredCloud->points[i].z;

            dst->points[i].r = filteredCloud->points[i].r;
            dst->points[i].g = filteredCloud->points[i].g;
            dst->points[i].b = filteredCloud->points[i].b;

            // cloud_normals -> Which you have already have; generated using pcl example code

            dst->points[i].curvature = cloud_normals->points[i].curvature;

            dst->points[i].normal_x = cloud_normals->points[i].normal_x;
            dst->points[i].normal_y = cloud_normals->points[i].normal_y;
            dst->points[i].normal_z = cloud_normals->points[i].normal_z;
        }

/*
        // created RandomSampleConsensus object and compute the appropriated model
        std::vector<int> inliers;
        pcl::copyPointCloud(*dst, *xyzrgb_only);
        pcl::copyPointCloud(*dst, *normal_only);
        pcl::SampleConsensusModelNormalSphere<pcl::PointXYZRGB, pcl::Normal>::Ptr
                model_sphere(new pcl::SampleConsensusModelNormalSphere<pcl::PointXYZRGB, pcl::Normal> (xyzrgb_only));
        model_sphere->setInputNormals(normal_only);
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model_sphere, 0.03);
        sac.computeModel();
        sac.getInliers(inliers);
        pcl::copyPointCloud(*xyzrgb_only, inliers, *final);*/

        // pointcloud source cloud
        // pointcloud output newCloud

        Eigen::Matrix4f flippy_trans = Eigen::Matrix4f::Identity();
        flippy_trans (1,1) = -1; // (row, col)
        flippy_trans (0,0) = -1; // (row, col)
        // std::cout << flippy_trans << std::endl;
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr spheres (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr newererCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
        //pcl::transformPointCloud( *final, *spheres, flippy_trans);
        pcl::transformPointCloud( *dst, *newererCloud, flippy_trans);

        cloudFile = "Captured_Frame" + to_string(fileCount) + ".pcd";
        spheresFile = "Sphere" + to_string(fileCount) + ".pcd";

        //==============================
        // Write PC to .pcd File Format
        //==============================
        // Take Cloud Data and write to .PCD File Format
        cout << "Generating PCD Point Cloud File... " << endl;
        //pcl::io::savePCDFileASCII(cloudFile, *spheres); // Input cloud to be saved to .pcd
        pcl::io::savePCDFileASCII(cloudFile, *newererCloud); // Input cloud to be saved to .pcd
        cout << cloudFile << " successfully generated. " << endl;

        //Load generated PCD file for viewing
        Load_PCDFile(cloudFile);
        //Load_PCDFile(spheresFile);
        fileCount++; // Increment File Name


        std::cout << "Exiting Program... " << endl;
        return EXIT_SUCCESS;
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception & e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
/*
//========================================
// userInput
// - Prompts user for a char to
// test for decision making.
// [y|Y] - Capture frame and save as .pcd
// [n|N] - Exit program
//========================================
bool CapturePointCLoud::userInput(void){

    bool setLoopFlag;
    bool inputCheck = false;
    char takeFrame; // Utilize to trigger frame capture from key press ('t')
    do {

        // Prompt User to execute frame capture algorithm
        cout << endl;
        cout << "Generate a Point Cloud? [y/n] ";
        cin >> takeFrame;
        cout << endl;
        // Condition [Y] - Capture frame, store in PCL object and display
        if (takeFrame == 'y' || takeFrame == 'Y') {
            setLoopFlag = true;
            inputCheck = true;
            takeFrame = 0;
        }
            // Condition [N] - Exit Loop and close program
        else if (takeFrame == 'n' || takeFrame == 'N') {
            setLoopFlag = false;
            inputCheck = true;
            takeFrame = 0;
        }
            // Invalid Input, prompt user again.
        else {
            inputCheck = false;
            cout << "Invalid Input." << endl;
            takeFrame = 0;
        }
    } while(inputCheck == false);

    return setLoopFlag;
}*/