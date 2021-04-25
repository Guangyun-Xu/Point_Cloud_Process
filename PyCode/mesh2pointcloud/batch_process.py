import os
import open3d as o3d
import shutil
import numpy as np
import pymeshlab

current_path = os.path.abspath(__file__)
father_path = os.path.abspath(os.path.dirname(current_path) + os.path.sep)
data_path = os.path.join(father_path, "Data/OBJ")
print(data_path)
paths = os.listdir(data_path)

for sub_path in paths:
    full_path = os.path.join(data_path, sub_path)
    files = os.listdir(full_path)
    ply_path = os.path.join(father_path, 'Data/PLY')
    face_a_path = os.path.join(father_path, 'Data/face_a')
    face_b_path = os.path.join(father_path, 'Data/face_b')

    if not os.path.exists(ply_path):
        os.mkdir(ply_path)

    if not os.path.exists(face_a_path):
        os.mkdir(face_a_path)

    if not os.path.exists(face_b_path):
        os.mkdir(face_b_path)

    for file in files:
        if ".obj" in file:
            mesh = o3d.io.read_triangle_mesh(os.path.join(full_path, file))
            mesh.compute_triangle_normals(normalized=True)
            # o3d.visualization.draw_geometries([mesh], point_show_normal=True)

            # 1.cover to pointcloud
            pc1 = mesh.sample_points_uniformly(number_of_points=100000)  # change number of points
            aabb = pc1.get_axis_aligned_bounding_box()
            aabb.color = (1, 0, 0)
            obb = pc1.get_oriented_bounding_box()
            obb.color = (0, 1, 0)
            pc_centre = aabb.get_center()
            centre_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=pc_centre)
            o3d.visualization.draw_geometries([pc1, aabb, centre_mesh, obb], point_show_normal=False)
            save_file = file.replace('.obj', '.ply')
            save_ply_path = os.path.join(full_path, save_file)
            print("save point cloud in: {}!".format(save_ply_path))
            o3d.io.write_point_cloud(save_ply_path, pc1)
            save_ply_path = os.path.join(ply_path, save_file)
            print("save point cloud in: {}!".format(save_ply_path))
            shutil.copyfile(os.path.join(full_path, save_file), save_ply_path)

            ms = pymeshlab.MeshSet()
            ms.load_new_mesh(os.path.join(full_path, save_file))

            ms.transform_align_to_principal_axis(freeze=False)

            ms.save_current_mesh('prcinpal_.ply')


            # 2.normal based filter, get face towards the z-axis
            def normal_filter(pc, oriention, threshold):
                """
                get face points toward target oriention.
                :param pc: o3d pointcloud
                :param oriention: target face oriention
                :return: o3d of target face pointcloud
                """

                xyz_points = np.asarray(pc.points)
                xyz_normals = np.asarray(pc.normals)
                angles = xyz_normals.dot(oriention)
                angle_mask = angles > threshold
                target_points = xyz_points[angle_mask]
                other_points = xyz_points[~angle_mask]
                target_normals = xyz_normals[angle_mask]
                other_normals = xyz_normals[~angle_mask]

                target_points_ = o3d.geometry.PointCloud()
                other_points_ = o3d.geometry.PointCloud()
                target_points_.points = o3d.utility.Vector3dVector(target_points)
                target_points_.normals = o3d.utility.Vector3dVector(target_normals)
                other_points_.points = o3d.utility.Vector3dVector(other_points)
                other_points_.normals = o3d.utility.Vector3dVector(other_normals)

                return target_points_, other_points_


            pc_face_z, other_points = normal_filter(pc1, [0, 0, 1], 0.5)
            save_face_a_path = os.path.join(face_a_path, save_file)
            o3d.io.write_point_cloud(save_face_a_path, pc_face_z)
            o3d.visualization.draw_geometries([pc_face_z], point_show_normal=True)

            # 3.get back face
            # remove face towards x and -x
            _, back_face_points = normal_filter(other_points, [1, 0, 0], 0.1)
            removed_points, back_face_points = normal_filter(back_face_points, [-1, 0, 0], 0.1)
            save_face_b_path = os.path.join(face_b_path, save_file)
            o3d.io.write_point_cloud(save_face_b_path, back_face_points)
            o3d.visualization.draw_geometries([back_face_points], point_show_normal=False)
