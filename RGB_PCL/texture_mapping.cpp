#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <math.h>
#include <vector>

#define PI 3.14159265

#define PANO_H_RES 6720
#define PANO_V_RES 3360
#define R PANO_H_RES/(2*PI)

#define VISUALIZE_POINT_SIZE 8

#define PCD_PATH "/Users/taojin/Desktop/RT_Scene_Capture/RGB_PCL/2.pcd"
#define IMG_PATH "/Users/taojin/Desktop/RT_Scene_Capture/RGB_PCL/2.jpg"

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

        // std::cout << pixel[0] << ", " << pixel[1] << std::endl;
        // std::cout << image_pixel[0] << ", " << image_pixel[1] << std::endl;

        cv::Vec3b pixel_color = image.at<cv::Vec3b>(image_pixel[1], image_pixel[0]);

        it->r = pixel_color[2];
        it->g = pixel_color[1];
        it->b = pixel_color[0];

    }
}

// Visualiza point cloud
pcl::visualization::PCLVisualizer::Ptr visualize_pcl(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, VISUALIZE_POINT_SIZE, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    return viewer;
}

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(PCD_PATH, *cloud);
    
    std::cout << "Point Cloud loaded successfully" << std::endl;
    std::cout << "Number of points in file: " << cloud->size() << std::endl;

    cv::Mat image = cv::imread(IMG_PATH);

    color_pcl(cloud, image);

    // pcl::io::savePCDFileASCII("color_pcl.pcd", *cloud);

    // Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = visualize_pcl(cloud);

    while(!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    
    return 0;
}
