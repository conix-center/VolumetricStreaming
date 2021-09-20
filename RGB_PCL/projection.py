import open3d as o3d
import numpy as np
from PIL import Image

file_path = "/Users/taojin/Desktop/RT_Scene_Capture/RGB_PCL/1.pcd"
LIDAR_HOR_RES = 2048
LIDAR_VER_RES = 128
LIDAR_FOV_UP = 45 / 180 * np.pi
LIDAR_FOV_DOWN = -45 / 180 * np.pi
LIDAR_FOV = LIDAR_FOV_UP + abs(LIDAR_FOV_DOWN)

# Generate Yaw and Pitch value of the cylindrical projected points
def yaw_pitch_generation(xyz_points):
    x = xyz_points[:,0]
    y = xyz_points[:,1]
    z = xyz_points[:,2]
    r = np.sqrt(x**2 + y**2 + z**2)
    yaw = np.arctan2(y, x)
    pitch = np.arcsin(z/r)

    return yaw, pitch, r

def project_uv(yaw, pitch):
    # convert to u,v coordinate
    u = np.floor(LIDAR_VER_RES * ((LIDAR_FOV_UP - pitch) / LIDAR_FOV))
    v = np.floor(LIDAR_HOR_RES * (yaw + np.pi) / (2 * np.pi))

    # clip values to [max/min]
    u = np.clip(u, 0, LIDAR_VER_RES - 1)
    v = np.clip(v, 0, LIDAR_HOR_RES - 1)

    u = np.nan_to_num(u, nan=0.0).astype(int)
    v = np.nan_to_num(v, nan=0.0).astype(int)

    return u, v

def visualize_uv(u, v, r):
    image = np.zeros((LIDAR_VER_RES, LIDAR_HOR_RES))
    
    for index in range(0, len(r)):
        # print(v[index], u[index])
        image[u[index], v[index]] = r[index]

    return image

def normalize_255(r):
    return (r - min(r)) / (max(r) - min(r)) * 255


def main():
    # import point cloud from file
    pcd = o3d.io.read_point_cloud(file_path)
    xyz_points = np.asarray(pcd.points)

    yaw, pitch, r = yaw_pitch_generation(xyz_points)

    r = normalize_255(r)

    u, v = project_uv(yaw, pitch)

    image = visualize_uv(u, v, r)
    print(max(r))

    img = Image.fromarray(np.uint8(image))
    img.show()

if __name__ == "__main__":
    main()
