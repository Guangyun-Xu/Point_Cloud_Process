import open3d as o3d
import numpy as np
import os
from normal_based_filter import normal_filter
from PyCode.mesh2pointcloud import mesh2pc

target_orientation = [0, 0, 1]
threshold = 0.1

mesh_path = "../mesh2pointcloud/Data/OBJ/135MC/db135-300-6.obj"
mesh = o3d.io.read_triangle_mesh(mesh_path)

pc = mesh2pc.mesh_2_pc(mesh, 10000, 'uniformly', True)  # mesh to pointcloud

# get face towards the z-axis
target_pc, other_points = normal_filter(pc, target_orientation, threshold)
o3d.visualization.draw_geometries([target_pc], point_show_normal=True)

# get back face
# remove face towards x and -x
_, back_face_points = normal_filter(other_points, [1, 0, 0], threshold)
removed_points, back_face_points = normal_filter(back_face_points, [-1, 0, 0], 0.1)
# save_face_b_path = os.path.join(face_b_path, save_file)
# o3d.io.write_point_cloud(save_face_b_path, back_face_points)
o3d.visualization.draw_geometries([back_face_points], point_show_normal=False)