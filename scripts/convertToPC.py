import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

RGB_PATH = "files/images/rgb_img_II/BGR00002.png"
DEPTH_PATH = "files/images/depth_img_II/D00002.npy"

color_raw = o3d.io.read_image(RGB_PATH) # Reads RGB image
depth_npy = np.load(DEPTH_PATH) # Reads Depth data
depth_raw  = o3d.geometry.Image(depth_npy) # Converts depth data into image format

rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(color_raw, depth_raw) # Creates RGBD image using TUM format

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)) # Creates Point Cloud from rgbd image
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) # Flip it, otherwise the pointcloud will be upside down

o3d.visualization.draw_geometries([pcd])