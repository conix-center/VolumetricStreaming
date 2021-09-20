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

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


#define VIDEO_DEVICE_ID 0

const int N_SCANS = 1;
const size_t UDP_BUF_SIZE = 65536;

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

void test_get_full_scan(std::shared_ptr<sensor::client> handle, 
                        LidarScan *scan, sensor::packet_format pf, 
                        ScanBatcher batch_to_scan,
                        size_t w, size_t h, int column_window_length,
                        XYZLut lut,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud) {
    while (true) {        
        LidarScan::Points clouds;

        // buffer to store raw packet data
        std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);

        for (int i = 0; i < N_SCANS;) {
            // wait until sensor data is available
            sensor::client_state st = sensor::poll_client(*handle);

            // check for error status
            if (st & sensor::CLIENT_ERROR)
                FATAL("Sensor client returned error state!");

            // check for lidar data, read a packet and add it to the current batch
            if (st & sensor::LIDAR_DATA) {
                if (!sensor::read_lidar_packet(*handle, packet_buf.get(), pf))
                    FATAL("Failed to read a packet of the expected size!");

                // batcher will return "true" when the current scan is complete
                if (batch_to_scan(packet_buf.get(), *scan)) {
                    // LidarScan provides access to azimuth block data and headers
                    auto n_invalid = std::count_if(
                        scan->headers.begin(), scan->headers.end(),
                        [](const LidarScan::BlockHeader& h) {
                            return h.status != 0xffffffff;
                        });
                    // retry until we receive a full set of valid measurements
                    // (accounting for azimuth_window settings if any)
                    if (n_invalid <= (int)w - column_window_length) i++;
                }
            }
        }

        // compute a point cloud using the lookup table
        clouds = ouster::cartesian(*scan, lut);

        point_cloud->clear();

        // for (int i = 0; i < clouds.rows(); i++) {
        for (int i = 0; i < 1; i++) {
            auto xyz = clouds.row(i);
            point_cloud->push_back(pcl::PointXYZRGB (xyz(0), xyz(1), xyz(2), 255, 255, 255));
            std::cout << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
            std::cout << xyz(0) * xyz(0) + xyz(1) * xyz(1) + xyz(2) * xyz(2) << std::endl;
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: rgb_pcl_streaming <sensor_hostname> <data_destination_ip>" << std::endl;

        return EXIT_FAILURE;
    }

    const std::string sensor_hostname = argv[1];
    const std::string data_destination = argv[2];

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";

    auto handle = sensor::init_client(sensor_hostname, data_destination, ouster::sensor::MODE_1024x10);
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

    // pre-compute a table for efficiently calculating point clouds from range
    XYZLut lut = ouster::make_xyz_lut(info);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    test_get_full_scan(handle, &scan, pf, batch_to_scan, w, h, column_window_length, lut, point_cloud);

    return 0;
}
