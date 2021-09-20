#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#define PCD_PATH_A "/Users/taojin/Desktop/RT_Scene_Capture/rgb_pcl_streaming/test_files/1.pcd"
#define PCD_PATH_B "/Users/taojin/Desktop/RT_Scene_Capture/rgb_pcl_streaming/test_files/2.pcd"

int main() {
    float resolution = 0.08f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(PCD_PATH_A, *cloudA);
    pcl::io::loadPCDFile(PCD_PATH_B, *cloudB);
    
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();

    octree.switchBuffers();

    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    // for (std::size_t i = 0; i < newPointIdxVector.size (); ++i) {
    //     std::cout << i << "# Index:" << newPointIdxVector[i]
    //             << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
    //             << (*cloudB)[newPointIdxVector[i]].y << " "
    //             << (*cloudB)[newPointIdxVector[i]].z << std::endl;

    //     std::cout << i << "# Index:" << newPointIdxVector[i]
    //             << "  Point:" << (*cloudA)[newPointIdxVector[i]].x << " "
    //             << (*cloudA)[newPointIdxVector[i]].y << " "
    //             << (*cloudA)[newPointIdxVector[i]].z << std::endl;
    // }

    std::cout << "size: " << newPointIdxVector.size() << std::endl;

    return 1;
}
