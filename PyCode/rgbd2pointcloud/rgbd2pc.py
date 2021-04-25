"""
# =================
# Convert pairwise RGB-D images to colored point cloud
# Author: Guangyun Xu
# Date: 10th Jan. 2021
# ==============================
"""
from PIL import Image
import numpy as np
import open3d as o3d


def dpt_2_cld(dpt, K):
    if len(dpt.shape) > 2:
        dpt = dpt[:, :, 0]
    max_dp = 1.5  # m
    min_dp = 0.1  # m
    cam_scale = K['camera_scale']
    msk_dp_min = dpt > (min_dp * cam_scale)
    msk_dp_max = dpt < (max_dp * cam_scale)
    msk_dp = msk_dp_max & msk_dp_min
    choose = msk_dp.flatten().nonzero()[0].astype(np.uint32)  # choose : 不为0的深度值的索引
    if len(choose) < 1:
        return None, None

    dpt_mskd = dpt.flatten()[choose][:, np.newaxis].astype(np.float32)  # 不为0的深度值
    xmap = np.array([[j for i in range(camera_params['img_width'])] for j in range(camera_params['img_height'])])
    ymap = np.array([[i for i in range(camera_params['img_width'])] for j in range(camera_params['img_height'])])
    xmap_mskd = xmap.flatten()[choose][:, np.newaxis].astype(np.float32)
    ymap_mskd = ymap.flatten()[choose][:, np.newaxis].astype(np.float32)

    pt2 = dpt_mskd / cam_scale
    cam_cx, cam_cy = K['x_offset'], K['y_offset']
    cam_fx, cam_fy = K['fx'], K['fy']
    pt0 = (ymap_mskd - cam_cx) * pt2 / cam_fx
    pt1 = (xmap_mskd - cam_cy) * pt2 / cam_fy
    cld = np.concatenate((pt0, pt1, pt2), axis=1)
    return cld, choose


# load image
color_image_path = "data/texture2.jpg"
depth_image_path = "data/depth2.png"

with Image.open(depth_image_path) as di:
    depth_image = np.array(di)
with Image.open(color_image_path) as ci:
    color_image = np.array(ci)/255
    channel = len(color_image.shape)
    if channel < 3:
        color_image = np.array([color_image, color_image, color_image])
    else:
        color_image = np.transpose(color_image, (2, 0, 1))

# camera config
camera_params = {
    # Camera/Frustum parameters
    'img_width': 671,
    'img_height': 502,
    'fx': 1122.375,
    'fy': 1122.375,
    'x_offset': 334.4445,
    'y_offset': 264.443075,
    'camera_scale': 10000
}

# cover depth to point cloud
cld, choose = dpt_2_cld(depth_image, camera_params)

# get pixel of each point
rgb_list = []
for image_channel in range(color_image.shape[0]):
    rgb_list.append(
        color_image[image_channel].flatten()[choose].astype(np.float32)
    )
rgb_pt = np.transpose(np.array(rgb_list), (1, 0)).copy()

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cld)
pcd.colors = o3d.utility.Vector3dVector(rgb_pt)

o3d.visualization.draw_geometries([pcd])
