cmake_minimum_required( VERSION 3.6 )

# Require C++11 (or later)
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Create Project
project( Sample )
add_executable( RS-Frame_Capture RS-Frame_Capture.cpp )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "RS-Frame_Capture" )

# Find Package
# librealsense2
set( realsense2_DIR "C:/Program Files/librealsense2/lib/cmake/realsense2" CACHE PATH "Path to librealsense2 config directory." )
find_package( realsense2 REQUIRED )

if( realsense2_FOUND )
  # Additional Include Directories
  include_directories( ${realsense_INCLUDE_DIR} )

  # Additional Dependencies
  target_link_libraries( RS-Frame_Capture ${realsense2_LIBRARY} )
endif()

find_package(PCL 1.8 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(RS-Frame_Capture ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


