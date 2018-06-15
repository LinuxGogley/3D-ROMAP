/***********************************************************
 * Author: Daniel Tran
 * Project: 3D-ROMAP Senior Project
 * Purpose: The following .cpp file will utilize the Intel
 *          realsense camera to stream and capture frame
 *          data of the environment. Color is then applied
 *          and a point cloud is generated and saved to
 *          a polygon file format (.PLY).
 *
 * Version 0.01 - Last Editted 6/14/18
 ***********************************************************/


#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <algorithm>            // std::min, std::max


// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char * argv[])
{
    // Create an OpenGL window for rendering
    window app(1280, 720, "3D-ROMAP Frame Capture Test");

    // Generate Object to manage viewing state
    glfw_state app_state;

    // Register callbacks for Point Cloud manipulation
    register_glfw_callbacks(app, app_state);

    //Point Cloud Object for PC calculation/Texture rendering
    rs2::pointcloud pc;

    // Points Object to display last generated pointcloud before frame drop
    rs2::points points;

    // Declare RealSense Pipeline (Device/Sensors) and stream with default config
    rs2::pipeline pipe;
    pipe.start();

    // While application is still running, update frame on window
    while (app) 
    {
        // Wait for the next set of frames from the camera (Blocking)
        auto frames = pipe.wait_for_frames();
        auto depth  = frames.get_depth_frame(); // Obtain depth from captured frame
	auto color  = frames.get_color_frame(); // Obtain color from captured frame
        
	// Generate pointcloud/texture mapping
        points = pc.calculate(depth);

        // Point Cloud object maps color from frame
        pc.map_to(color);

	// Generate a .PLY file format of captured point cloud
	points.rs2_export_to_ply(depth,"Captured_Frame.ply", color);

        // Upload the color frame to OpenGL
        app_state.tex.upload(color);

        // Generate the Point Cloud for viewing
        draw_pointcloud(app.width(), app.height(), app_state, points);
    }

    return 0;
}

