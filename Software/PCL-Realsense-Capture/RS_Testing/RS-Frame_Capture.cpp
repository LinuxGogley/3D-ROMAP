/***********************************************************
 * Author:  Daniel Tran
 *          Liam Gogley
 * 
 * Project: 3D-ROMAP Senior Project
 * Purpose: The following .cpp file will utilize the Intel
 *          realsense camera to stream and capture frame
 *          data of the environment. Color is then applied
 *          and a point cloud is generated and saved to
 *          a point cloud data format (.pcd).
 * 
 * Version 0.07 - Last Editted 7/30/18
 * 
 * Rev:     Implementation of RGB Texture function to map
 *          color to point cloud data.
 * 
 ***********************************************************/

#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
// Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
// OpenCV Headers
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

// Prototypes
void Load_PCDFile(void);


//==============
// RGB Texture
//==============
// Obtain RGB values from normals[u,v] to texture coordinates[w,h]
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width(); 
    int height = texture.get_height();
    
     // Convert normals[u v] to coordinates[x y]
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();
    int strides = y_value * texture.get_stride_in_bytes();
    int Text_Index =  (bytes + strides);
    
    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

// Convert Depth/RGB from Realsense to PCL Format
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    /*
    // Display RGB Color Image (Testing Purposes)
    Mat colorr(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", colorr);
    */
    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to PCL
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        // Mapping Depth Coordinates
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

int main() {

    //====================
    // Object Declaration
    //====================
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
   
    // Begin Stream with default configs
    pipe.start(cfg); 

    // Wait for frames from the camera to settle
    for (int i = 0; i < 30; i++) {
        auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
    }

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
    // Filter PointCloud (PassThrough Method)
    //========================================
    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
    Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
    Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
    Cloud_Filter.setFilterLimits (0.0, 1.0);      // Set accepted interval values
    Cloud_Filter.filter (*newCloud);              // Filtered Cloud Outputted

    //==============================
    // Write PC to .pcd File Format
    //==============================
    // Take Cloud Data and write to .PCD File Format
    cout << "Generating PCD Point Cloud File... " << endl;
    pcl::io::savePCDFileASCII("Captured_Frame.pcd", *newCloud);
    cout << "Captured_Frame.pcd successfully generated. " << endl; 

    
    // Load generated PCD file for viewing
    Load_PCDFile();
   

  
    return EXIT_SUCCESS;
}

void Load_PCDFile(void)
{
    // Generate object to store cloud in .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::io::loadPCDFile ("Captured_Frame.pcd", *cloudView); // Load .pcd File
    
    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Room Map"
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Room Map"));
    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0); 
    // Add generated point cloud and identify with string "Room Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudView, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // Viewer Properties
    //viewer->addCoordinateSystem (1.0); // Display XYZ axis
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing
   
    
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
