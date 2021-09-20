import open3d as o3d
import cv2

FILE_PATH = "/Users/taojin/Desktop/RT_Scene_Capture/RGB_PCL/1.pcd"

def crop_pcl(pcd):
    

    return cropped_pcd

def main():
    # import point cloud from file
    pcd = o3d.io.read_point_cloud(FILE_PATH)

    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()