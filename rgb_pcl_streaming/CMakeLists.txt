cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(rgb_pcl_streaming_project)

find_package(PCL 1.11 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(include/optional-lite include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${JSONCPP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(rgb_pcl_streaming
  src/main_multithread.cpp
  src/pcl_camera_fusion.cpp
  src/client.cpp
  src/lidar_scan.cpp
  src/types.cpp
  src/netcompat.cpp
  include/pcl_camera_fusion.h
  include/ouster/client.h
  include/ouster/lidar_scan.h
  include/ouster/types.h
  include/ouster/impl/netcompat.h
)

target_link_libraries(rgb_pcl_streaming 
                      ${PCL_LIBRARIES} 
                      ${PCL_COMMON_LIBRARIES} 
                      ${PCL_IO_LIBRARIES} 
                      ${OpenCV_LIBS} 
                      ${JSONCPP_LIBRARIES})

# add_executable(bg_filter
#   src/background_filter.cpp
# )
                    
# target_link_libraries(bg_filter 
#                       ${PCL_LIBRARIES} 
#                       ${PCL_COMMON_LIBRARIES} 
#                       ${PCL_IO_LIBRARIES})


# add_executable(lidar_test
#   src/lidar_test.cpp
#   src/pcl_camera_fusion.cpp
#   src/client.cpp
#   src/lidar_scan.cpp
#   src/types.cpp
#   src/netcompat.cpp
#   include/pcl_camera_fusion.h
#   include/ouster/client.h
#   include/ouster/lidar_scan.h
#   include/ouster/types.h
#   include/ouster/impl/netcompat.h
# )

# target_link_libraries(lidar_test 
#                       ${PCL_LIBRARIES} 
#                       ${PCL_COMMON_LIBRARIES} 
#                       ${PCL_IO_LIBRARIES} 
#                       ${OpenCV_LIBS} 
#                       ${JSONCPP_LIBRARIES})
