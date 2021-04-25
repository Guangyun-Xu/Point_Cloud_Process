"""
# =================
# select points by normal
# Author: Guangyun Xu
# Date: 25th Apr. 2021
# ==============================
"""
import open3d as o3d
import numpy as np
from mesh2pointcloud import mesh2pc

def normal_filter(pointcloud, target_orient, threshold):
    """
    get points of surfaces which toward target target orientation.
    :param threshold: the threshold value of the angle between the
    target orientation and the surface normal.
    :param pointcloud: o3d pointcloud
    :param target_orient: target face orientation
    :return: o3d of target face pointcloud
    """

    xyz_points = np.asarray(pointcloud.points)
    xyz_normals = np.asarray(pointcloud.normals)
    projection = xyz_normals.dot(target_orient)
    projection_mask = projection > threshold  #
    target_points = xyz_points[projection_mask]
    other_points = xyz_points[~projection_mask]
    target_normals = xyz_normals[projection_mask]
    other_normals = xyz_normals[~projection_mask]

    target_points_ = o3d.geometry.PointCloud()
    other_points_ = o3d.geometry.PointCloud()
    target_points_.points = o3d.utility.Vector3dVector(target_points)
    target_points_.normals = o3d.utility.Vector3dVector(target_normals)
    other_points_.points = o3d.utility.Vector3dVector(other_points)
    other_points_.normals = o3d.utility.Vector3dVector(other_normals)

    return target_points_, other_points_


if __name__ == '__main__':

    target_orientation = [0, 0, 1]
    threshold = 0.1

    mesh_path = "../mesh2pointcloud/Data/61003522db216mb-600001.obj"
    mesh = o3d.io.read_triangle_mesh(mesh_path)

    pc = mesh2pc.mesh_2_pc(mesh, 10000, 'uniformly', True)  # mesh to pointcloud
    target_pc, _ = normal_filter(pc, target_orientation, threshold)
    o3d.visualization.draw_geometries([target_pc], point_show_normal=True)
