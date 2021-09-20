#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <math.h>
#include <chrono>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "pcl_camera_fusion.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


#define VIDEO_DEVICE_ID 0

using namespace ouster;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: rgb_pcl_streaming <sensor_hostname_1> <sensor_hostname_2>" << std::endl;

        return EXIT_FAILURE;
    }

    // Camera Init
    cv::VideoCapture cap;
    cap.open(VIDEO_DEVICE_ID);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open capture device." << std::endl;
        return 1;
    }
    std::cout << "Camera opened successfully" << std::endl;
    cv::Mat frame;

    const std::string sensor_hostname_1 = argv[1];
    const std::string sensor_hostname_2 = argv[2];
    const std::string data_destination = "";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // float resolution = 0.05f;
    // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(resolution);

    // Multithreading starts here
    // std::thread lidar_scan(get_full_scan, handle, &scan, pf, batch_to_scan, w, h, column_window_length, lut, point_cloud, transformed_cloud, voxel_cloud, &octree);
    std::thread lidar_scan_1(get_full_scan, sensor_hostname_1, data_destination, point_cloud_1, 1);
    std::thread lidar_scan_2(get_full_scan, sensor_hostname_2, data_destination, point_cloud_2, 2);
    // std::thread camera_capture(cam_capture_frame, cap, &frame);

    // visualizer(voxel_cloud, cap, frame);
    visualizer(point_cloud_1, point_cloud_2, cap, frame);
    
    return EXIT_SUCCESS;
}
