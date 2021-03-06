project(realsense-testing)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "-std=c++11")

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "RS-Frame_Capture" )
add_executable (RS-Frame_Capture RS-Frame_Capture.cpp)

# Find Packages

#==========
# OpenCV
#==========
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

#==============
# Librealsense
#==============
set( realsense2_DIR "C:/Program Files/librealsense2/lib/cmake/realsense2" CACHE PATH "Path to librealsense2 config directory." )
find_package( realsense2 REQUIRED )

if( realsense2_FOUND )
  # Additional Include Directories
  include_directories( ${realsense_INCLUDE_DIR} )

  # Additional Dependencies
  target_link_libraries( RS-Frame_Capture ${realsense2_LIBRARY} )
endif()

#==========
# PCL v1.8
#==========
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(RS-Frame_Capture ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries (RS-Frame_Capture ${PCL_LIBRARIES})

#===================
# PCL Visualization
#===================
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (RS-Frame_Capture ${PCL_LIBRARIES})

target_link_libraries (RS-Frame_Capture ${PCL_LIBRARIES} librealsense2.so ${OpenCV_LIBS})


