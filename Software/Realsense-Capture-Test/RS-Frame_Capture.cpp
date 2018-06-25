/***********************************************************
 * Author:  Daniel Tran
 *          Liam Gogley
 * Project: 3D-ROMAP Senior Project
 * Purpose: The following .cpp file will utilize the Intel
 *          realsense camera to stream and capture frame
 *          data of the environment. Color is then applied
 *          and a point cloud is generated and saved to
 *          a polygon file format (.PLY).
 *
 * Version 0.02 - Last Editted 6/23/18
 ***********************************************************/


#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

int main(int argc, char * argv[])
{
    //Point Cloud Object for PC calculation/Texture rendering
    rs2::pointcloud pc;

    // Points Object to display last generated pointcloud before frame drop
    rs2::points points;

    // Declare RealSense Pipeline (Device/Sensors) and stream with default config
    rs2::pipeline pipe;
    pipe.start();

    // Wait for frames from the camera to settle
    for (auto i = 0; i < 30; ++i) pipe.wait_for_frames();
    auto frames = pipe.wait_for_frames();
    auto depth  = frames.get_depth_frame(); // Obtain depth from captured frame
    auto color  = frames.get_color_frame(); // Obtain color from captured frame
    
    // Point Cloud object maps color from frame
    pc.map_to(color);
        
    // Generate pointcloud/texture mapping
    points = pc.calculate(depth);

    // Generate a .PLY file format of captured point cloud
    points.export_to_ply("Captured_Frame.ply", color);

    return 0;
}

