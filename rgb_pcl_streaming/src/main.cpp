#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "pcl_camera_fusion.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <math.h>
#include <vector>
#include <chrono>

// #define PI 3.14159265
// #define PANO_H_RES 1920
// #define PANO_V_RES 960
// #define R PANO_H_RES/(2*PI)

#define VISUALIZE_POINT_SIZE 8
#define VIDEO_DEVICE_ID 0

using namespace ouster;

sensor::sensor_info sensor_show_metadata(std::string metadata) {
    sensor::sensor_info info = sensor::parse_metadata(metadata);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    int column_window_length =
        (column_window.second - column_window.first + w) % w + 1;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h
              << "\n  Column window:     [" << column_window.first << ", "
              << column_window.second << "]" << std::endl;

    return info;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: rgb_pcl_streaming <sensor_hostname> <data_destination_ip>" << std::endl;

        return EXIT_FAILURE;
    }

    const std::string sensor_hostname = argv[1];
    const std::string data_destination = argv[2];

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";

    auto handle = sensor::init_client(sensor_hostname, data_destination, ouster::sensor::MODE_1024x20);
    if (!handle) FATAL("Failed to connect");
    std::cerr << "ok" << std::endl;

    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = sensor::get_metadata(*handle);

    auto info = sensor_show_metadata(metadata);
    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;
    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    int column_window_length =
        (column_window.second - column_window.first + w) % w + 1;

    // A LidarScan holds lidar data for an entire rotation of the device
    LidarScan scan = LidarScan{w, h};

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pf = sensor::get_format(info);
    ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

    std::cerr << "Capturing points... ";

    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

    // pre-compute a table for efficiently calculating point clouds from range
    XYZLut lut = ouster::make_xyz_lut(info);

    // Camera Init
    cv::VideoCapture cap;
    cap.open(VIDEO_DEVICE_ID);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open capture device." << std::endl;
        return 1;
    }
    std::cout << "Camera opened successfully" << std::endl;
    cv::Mat frame;

    bool initialized = false;

    while (1) {
        pcl::visualization::PCLVisualizer::Ptr viewer;
        viewer = visualizer_init();

        while(!viewer->wasStopped()) {
            auto begin = std::chrono::high_resolution_clock::now();

            if (initialized) {
                viewer->removePointCloud("cloud");
            }
            
            get_full_scan(handle, packet_buf, &scan, pf, batch_to_scan, w, h, column_window_length);

            LidarScan::Points clouds;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            // compute a point cloud using the lookup table
            clouds = ouster::cartesian(scan, lut);

            for (int i = 0; i < clouds.rows(); i++) {
                auto xyz = clouds.row(i);
                point_cloud->push_back(pcl::PointXYZRGB (xyz(0), xyz(1), xyz(2), 255, 255, 255));
            }

            // Begin Fusion
            cap.read(frame);
            if (frame.empty()) {
                std::cerr << "Receiced empty video frame" << std::endl;
            }

            color_pcl(point_cloud, frame);

            viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud, "cloud");
            if (!initialized) {
                initialized = true;
            }
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZE_POINT_SIZE, "cloud");

            viewer->spinOnce();

            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
            printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);
        }

    }

    return EXIT_SUCCESS;
}
