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
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace ouster;

std::vector<float> cartesian_to_spherical(std::vector<float> point_xyz);

std::vector<int> spherical_to_equirectangular(std::vector<float> point_spherical);

void color_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat image);

pcl::visualization::PCLVisualizer::Ptr visualizer_init();

void FATAL(const char* msg);

void pcl_change_detector(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// void get_full_scan(std::shared_ptr<sensor::client> handle, 
//                         LidarScan *scan, sensor::packet_format pf, 
//                         ScanBatcher batch_to_scan,
//                         size_t w, size_t h, int column_window_length,
//                         XYZLut lut,
//                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
//                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud,
//                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud,
//                         pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> *octree);
void get_full_scan(std::string sensor_hostname, std::string data_destination, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, int lidar_num);

void cam_capture_frame(cv::VideoCapture cap, cv::Mat *frame);

void visualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_2, cv::VideoCapture cap, cv::Mat frame);
// void visualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, cv::Mat frame);

void transform_pcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud);

void voxelization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud);
