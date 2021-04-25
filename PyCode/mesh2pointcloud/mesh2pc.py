import open3d as o3d
import os
import shutil


def mesh_2_pc(mesh, number_of_points=10000, mode='uniformly', is_normal=True):
    """
    cover mesh to pointcloud.
    :param is_normal:
    :param mesh:
    :param number_of_points:
    :param mode: uniformly or poisson disk
    :return:  o3d pointcloud
    """
    if is_normal:
        mesh.compute_triangle_normals(normalized=True)
    if mode == 'uniformly':
        pc_ = mesh.sample_points_uniformly(number_of_points=number_of_points)
    elif mode == 'poisson_disk':
        pc_ = mesh.sample_points_poisson_disk(number_of_points=number_of_points)

    return pc_


if __name__ == '__main__':

    is_batch_process = True
    is_visualize_mesh = True
    is_visualize_points = True
    sample_mode = 'uniformly'
    num_of_points = 1000

    if is_batch_process:
        current_path = os.path.abspath(__file__)
        father_path = os.path.abspath(os.path.dirname(current_path) + os.path.sep)
        data_path = os.path.join(father_path, "Data\OBJ")
        print("data path:{}".format(data_path))
        paths = os.listdir(data_path)

        for sub_path in paths:
            full_path = os.path.join(data_path, sub_path)
            files = os.listdir(full_path)
            ply_path = os.path.join(father_path, 'Data\PLY')
            if not os.path.exists(ply_path):
                os.mkdir(ply_path)

            for file in files:
                if file[-4:] == ".obj":
                    mesh_path = os.path.join(full_path, file)
                    print("mesh file          : {}".format(mesh_path))
                    mesh = o3d.io.read_triangle_mesh(mesh_path)
                    pc = mesh_2_pc(mesh, number_of_points=100000, mode=sample_mode)
                    save_file = file.replace('.obj', '.ply')
                    save_ply_path = os.path.join(full_path, save_file)
                    print("save point cloud in: {}!".format(save_ply_path))
                    o3d.io.write_point_cloud(save_ply_path, pc)
                    save_ply_path = os.path.join(ply_path, save_file)
                    print("save point cloud in: {}!".format(save_ply_path))
                    shutil.copyfile(os.path.join(full_path, save_file), save_ply_path)

    else:
        mesh_path = "Data/61003522db216mb-600001.obj"
        mesh = o3d.io.read_triangle_mesh(mesh_path)

        if is_visualize_mesh:
            o3d.visualization.draw_geometries([mesh])

        pc_ = mesh_2_pc(mesh, number_of_points=num_of_points, mode=sample_mode)
        if is_visualize_points:
            o3d.visualization.draw_geometries([pc_])

        o3d.io.write_point_cloud(mesh_path.replace(".obj", ".ply"), pc_)
