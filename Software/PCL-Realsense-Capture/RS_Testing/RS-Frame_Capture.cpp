/***********************************************************
 * Author:  Daniel Tran
 *	        Liam Gogley
 * 
 * Project: 3D-ROMAP Senior Project
 * Purpose: The following .cpp file will utilize the Intel
 *          realsense camera to stream and capture frame
 *          data of the environment. Color is then applied
 *          and a point cloud is generated and saved to
 *          a polygon file format (.PLY).
 *
 * Version 0.03 - Last Editted 6/20/18
 ***********************************************************/


#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

using namespace std;

void Load_PCDFile(void);

int main(int argc, char * argv[])
{
     pcl::PointCloud<pcl::PointXYZ> cloud; // Cloud from PCL

    //Point Cloud Object for PC calculation/Texture rendering
    rs2::pointcloud pc;

    // Points Object to display last generated pointcloud before frame drop
    rs2::points points;

    // Declare RealSense Pipeline (Device/Sensors) and stream with default config
    rs2::pipeline pipe;
    pipe.start();

    // Wait for the next set of frames from the camera (Blocking)
    auto frames = pipe.wait_for_frames();

    // Depth and color capture
    auto depth  = frames.get_depth_frame(); // Obtain depth from captured frame
    auto color  = frames.get_color_frame(); // Obtain color from captured frame
        
    // Generate pointcloud/texture mapping
    points = pc.calculate(depth);

    // Point Cloud object maps color from frame
    pc.map_to(color);

    // Convert data captured from Realsense camera to PCL
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud.width = sp.width();
    cloud.height = sp.height();
    cloud.is_dense = false;
    cloud.points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud.points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    // Take Cloud Data and write to .PCD File Format
    cout << "Generating PCD Point Cloud File... " << endl;
    pcl::io::savePCDFileASCII("Captured_Frame.pcd", cloud);
    cout << "Captured_Frame.pcd successfully generated. " << endl; 
    
    Load_PCDFile(); // Load te PCD File and display the data points (Testing)
   
    return 0;
}
// Function will load PCD file generated and display data points of cloud
void Load_PCDFile(void)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("Captured_Frame.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    cout << "Loaded " << cloud->width * cloud->height
         << " data points from Captured_Frame.pcd with the following fields: "
         << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cout << "    " << cloud->points[i].x
             << " "    << cloud->points[i].y
             << " "    << cloud->points[i].z 
             << std::endl;
    }
}