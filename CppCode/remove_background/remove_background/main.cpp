#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // 1.load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr background_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::string background_path ="/home/yumi/Desktop/SampleDate/0909/background.ply";
    std::string scene_path = "/home/yumi/Desktop/SampleDate/0909/scene.ply";
    pcl::io::loadPLYFile(background_path, *background_cloud);
    pcl::io::loadPLYFile(scene_path, *scene_cloud);

    // 2.voxelization
    float resolution = 0.03f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> background_tree (resolution);
    background_tree.setInputCloud (background_cloud);
    background_tree.addPointsFromInputCloud ();
    background_tree.switchBuffers ();

    background_tree.setInputCloud (scene_cloud);
    background_tree.addPointsFromInputCloud ();

    // 3.get new point set
    std::vector<int> newPointIdxVector;
    background_tree.getPointIndicesFromNewVoxels(newPointIdxVector);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    for (size_t i = 0; i < newPointIdxVector.size(); i++)
    {
        inliers->indices.push_back(newPointIdxVector[i]);
    }

    // 4.extract
    pcl::PointCloud<pcl::PointXYZ>::Ptr foreground (new pcl::PointCloud<pcl::PointXYZ> );
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(scene_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*foreground);

    // 5.remove outlier
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (foreground);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.01);
    sor.filter (*foreground);

    // 6.save
    std::string foreground_path = "/home/yumi/Desktop/SampleDate/0909/foreground.pcd";
    pcl::PCDWriter writer_pcd;
    writer_pcd.write(foreground_path, *foreground);

    // 7.visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(foreground, "1");
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }




}
