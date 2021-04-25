#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// camera parameter
const double camera_factor = 10;
const double camera_cx = 334.4445;
const double camera_cy = 264.443075;
const double camera_fx = 1122.375;
const double camera_fy = 1122.375;
const int image_width = 671;
const int image_hight = 502;

int main()
{
    // load depth image
    std::string depth_path = "../000000.png";
    cv::Mat depth;
    depth = cv::imread(depth_path, -1);  // "-1":read original image with no modify
    std::cout<< "image size:" <<depth.size<<"\n"<<std::endl;
    std::cout<< "image height:"<<depth.cols<<"\n"<<std::endl;
    std::cout<< "image width:"<<depth.rows<<"\n"<<std::endl;

    // convert to point
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud (new pcl::PointCloud<pcl::PointXYZ>);

    for(int m=0; m<(depth.rows); m++)
    {
        for(int n=0; n<depth.cols; n++)
        {
            // get value of (m,n)
            ushort d = depth.ptr<ushort>(m)[n];  // (rows)[cols]

            if (d<1000 || d>14000)
                continue;

            pcl::PointXYZ p;

            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            pointcloud->points.push_back(p);
        }
    }

    pcl::PLYWriter ply_writer;
    ply_writer.write("../pointcloud.ply", *pointcloud);


    pcl::visualization::CloudViewer viewer ("show cloud");
    viewer.showCloud(pointcloud);

    while(!viewer.wasStopped())
    {

    }

}
