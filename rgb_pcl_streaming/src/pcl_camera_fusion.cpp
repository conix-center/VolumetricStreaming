#include "pcl_camera_fusion.h"

#include <vector>
#include <mutex>
#include <chrono>
#include <thread>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#define PI 3.14159265
#define PANO_H_RES 1920
#define PANO_V_RES 960
#define R PANO_H_RES/(2*PI)

#define VISUALIZE_POINT_SIZE 5

const int N_SCANS = 1;
const size_t UDP_BUF_SIZE = 65536;

static std::mutex lidar_lock_1;
static std::mutex lidar_lock_2;
// static std::mutex camera_lock;

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

std::vector<float> cartesian_to_spherical(std::vector<float> point_xyz) {
    float x = point_xyz[0];
    float y = point_xyz[1];
    float z = -point_xyz[2];

    float r = sqrt(x * x + y * y + z * z);
    float theta = -atan2(y, x);
    float phi = (90 * PI / 180) - acos(z / r);

    std::vector<float> point_spherical = {theta, phi, r};

    return point_spherical;
}

std::vector<int> spherical_to_equirectangular(std::vector<float> point_spherical) {
    if (point_spherical[0] != point_spherical[0]) {
        return {0, 0};
    }

    int x = floor(R * point_spherical[0] * cos(point_spherical[1]));
    int y = floor(R * point_spherical[1]);

    std::vector<int> pixel = {x, y};

    return pixel;
}

void color_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat image) {
    // Check if point lies in the camera FOV
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        std::vector<float> point_xyz = {it->x, it->y, it->z};

        if (sqrt(point_xyz[0] *  point_xyz[0] + point_xyz[1] * point_xyz[1] + point_xyz[2] * point_xyz[2]) < 0.3) {
            continue;
        }

        std::vector<float> point_spherical = cartesian_to_spherical(point_xyz);
        std::vector<int> pixel = spherical_to_equirectangular(point_spherical);

        std::vector<int> image_pixel = {pixel[0] + PANO_H_RES / 2, pixel[1] + PANO_V_RES / 2};

        // std::cout << "pixel[0]: " << image_pixel[0] << ", pixel[1]: " << image_pixel[1] << std::endl;

        cv::Vec3b pixel_color = image.at<cv::Vec3b>(image_pixel[1], image_pixel[0]);

        it->r = pixel_color[2];
        it->g = pixel_color[1];
        it->b = pixel_color[0];

    }
}

// Visualiza point cloud
pcl::visualization::PCLVisualizer::Ptr visualizer_init() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZE_POINT_SIZE, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    return viewer;
}

void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

void transform_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud) {
    Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
    transform_matrix.translation() << 0.0, 0.0, -0.185;

    // std::cout << "Transformation Matrix: \n" << transform_matrix.matrix() << std::endl;

    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);
}

void voxelization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_grid.filter(*voxel_cloud);
}

void pcl_change_detector(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    octree->switchBuffers();

    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();

    std::vector<int> newPointIdxVector;

    octree->getPointIndicesFromNewVoxels(newPointIdxVector, 2);

    std::cout << "size: " << newPointIdxVector.size() << std::endl;

    for (std::size_t i = 0; i < newPointIdxVector.size(); i++) {
        (*cloud)[newPointIdxVector[i]].r = 255;
        (*cloud)[newPointIdxVector[i]].g = 0;
        (*cloud)[newPointIdxVector[i]].b = 0;
    }
}

// void get_full_scan(std::shared_ptr<sensor::client> handle, 
//                         LidarScan *scan, sensor::packet_format pf, 
//                         ScanBatcher batch_to_scan,
//                         size_t w, size_t h, int column_window_length,
//                         XYZLut lut,
//                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
//                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud,
//                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud,
//                         pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree) {
void get_full_scan(std::string sensor_hostname, std::string data_destination, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, int lidar_num) {
    int write_pcd_1 = false;
    int write_pcd_2 = false;
    // Init lidar
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
                if (batch_to_scan(packet_buf.get(), scan)) {
                    // LidarScan provides access to azimuth block data and headers
                    auto n_invalid = std::count_if(
                        scan.headers.begin(), scan.headers.end(),
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
        clouds = ouster::cartesian(scan, lut);

        if (lidar_num == 1) {
            lidar_lock_1.lock();
        } else {
            lidar_lock_2.lock();
        }

        point_cloud->clear();

        if (lidar_num == 1) {
            for (int i = 0; i < clouds.rows(); i++) {
                auto xyz = clouds.row(i);
                point_cloud->push_back(pcl::PointXYZRGB (xyz(0), xyz(1), xyz(2), 255, 0, 0));
            }
        } else {
            for (int i = 0; i < clouds.rows(); i++) {
                auto xyz = clouds.row(i);
                point_cloud->push_back(pcl::PointXYZRGB (xyz(0), xyz(1), xyz(2), 0, 255, 0));
            }
        }
        

        // pcl_change_detector(octree, point_cloud);

        // transform_pcl(point_cloud, transformed_cloud);

        // voxelization(transformed_cloud, voxel_cloud);

        if (lidar_num == 1) {
            if (write_pcd_1 == false) {
                pcl::io::savePCDFileBinary("cloud_1.pcd", *point_cloud);
                write_pcd_1 = true;
                std::cout << "cloud_1 wrote successfully" << std::endl;
            }
            
            lidar_lock_1.unlock();
        } else {
            if (write_pcd_2 == false) {
                pcl::io::savePCDFileBinary("cloud_2.pcd", *point_cloud);
                write_pcd_2 = true;
                std::cout << "cloud_2 wrote successfully" << std::endl;
            }

            lidar_lock_2.unlock();
        }
        
    }
}

// void cam_capture_frame(cv::VideoCapture cap, cv::Mat *frame) {
//     while (true) {
//         camera_lock.lock();

//         cap.read(*frame);
//         if (frame->empty()) {
//             std::cerr << "Receiced empty video frame" << std::endl;
//         }
//         camera_lock.unlock();

//         std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
// }

void visualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_2, cv::VideoCapture cap, cv::Mat frame) {
    Eigen::Matrix4f transform_0_1;
    transform_0_1 <<    0.180898,  -0.983039, -0.0302977,    2.61152,
                        0.983301,     0.1814, -0.0147428,   -2.84651,
                        0.0199887, -0.0271246,   0.999431,   0.129603,
                                0,          0,          0,          1;
    std::cout << "\n" << transform_0_1 << std::endl;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = visualizer_init();

    while(!viewer->wasStopped()) {
        // auto begin = std::chrono::high_resolution_clock::now();

        std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_list;

        if (std::try_lock(lidar_lock_1, lidar_lock_2)) {
            pcl::transformPointCloud(*point_cloud_1, *point_cloud_1, transform_0_1);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr stiched_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
            *stiched_pcl += (*point_cloud_1 + *point_cloud_2);

            cloud_list.push_back(point_cloud_1);

            if (!viewer->updatePointCloud(stiched_pcl, "cloud")) {
                viewer->addPointCloud(stiched_pcl, "cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZE_POINT_SIZE, "cloud");
            }

            lidar_lock_1.unlock();
            lidar_lock_2.unlock();
        }

        // auto end = std::chrono::high_resolution_clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
        // printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);

        viewer->spinOnce(100);
    }
}
