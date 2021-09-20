cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(rgb_pcl_project)

# set(PCL_DIR "/opt/homebrew/Cellar/pcl/1.12.0/share/pcl-1.12/")

find_package(PCL 1.11 REQUIRED COMPONENTS common io visualization)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(streaming streaming.cpp)
target_link_libraries(streaming ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS})
