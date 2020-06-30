#include <iostream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <Eigen/Core>
#include <pcl/common/centroid.h>
#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    //load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::string fileName ="/home/yumi/Desktop/SampleDate/cloud_cluster_0.ply";
    pcl::io::loadPLYFile(fileName, *cloud);
    std::cout<<cloud->size()<<std::endl;

    //compute centroid
    Eigen::Vector4f pcaCentroid;    //0 1 2 -> x y z , 3 -> set to 1,allows to transform the centroid vector with 4x4 matrices.
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    std::cout<<"centroid:"<<pcaCentroid<<std::endl;

    //compute covariance matrix
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    std::cout<<"covariance:\n"<<covariance<<std::endl;

    //compute eigen vectors and eigen values of covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();  // first col is main direction
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    std::cout<<"eigen vectors:\n"<<eigenVectorsPCA<<std::endl;
    std::cout<<"eigen values:\n"<<eigenValuesPCA<<std::endl;

//    //transform point cloud
//    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
//    transformMatrix.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
//    transformMatrix.block<3, 1>(0, 3) = -1.0f*(transformMatrix.block<3, 3>(0, 0)) * (pcaCentroid.head<3>());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*cloud, *transformedCloud, transformMatrix);

    //main direction in cloud
    pcl::PointXYZ c;    //  start point
    c.x = pcaCentroid(0);
    c.y = pcaCentroid(1);
    c.z = pcaCentroid(2);
    std::cout<<"c:\n"<<c<<std::endl;


    pcl::PointXYZ pcZ;  // end point
    pcZ.x = 1000*eigenVectorsPCA(0, 0) + c.x;
    pcZ.y = 1000*eigenVectorsPCA(1, 0) + c.y;
    pcZ.z = 1000*eigenVectorsPCA(2, 0) + c.z;
    std::cout<<"pcZ:\n"<<pcZ<<std::endl;

    pcl::PointXYZ pcY;
    pcY.x = 1000*eigenVectorsPCA(0, 1) + c.x;
    pcY.y = 1000*eigenVectorsPCA(1, 1) + c.y;
    pcY.z = 1000*eigenVectorsPCA(2, 1) + c.z;

    pcl::PointXYZ pcX;
    pcX.x = 1000*eigenVectorsPCA(0, 2) + c.x;
    pcX.y = 1000*eigenVectorsPCA(1, 2) + c.y;
    pcX.z = 1000*eigenVectorsPCA(2, 2) + c.z;
    std::cout<<"pcY:\n"<<pcY<<std::endl;
    std::cout<<"pcX:\n"<<pcX<<std::endl;
    //viaualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer1->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(other_point, 0, 255, 0);
    viewer1->addPointCloud<pcl::PointXYZ>(cloud, red, "1");
    //viewer1->addPointCloud<pcl::PointXYZ>(other_point, green, "2");

    viewer1->addArrow(pcZ, c, 255.0, 0.0, 0.0, false, "Z");
    viewer1->addArrow(pcX, c, 0.0, 255.0, 0.0, false, "X");
    viewer1->addArrow(pcY, c, 0.0, 0.0, 255.0, false, "Y");

    viewer1->addCoordinateSystem(100);
    while(!viewer1->wasStopped())
    {
        viewer1->spinOnce(1000);//等待100ms
    }

    return 0;

}
