#include <iostream>
#include <Eigen/Dense>

#include <vector>
#include <thread>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

pcl::PointXYZ getEndPoint(pcl::PointXYZ statPoint, float yaw, float length)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 
    transform.translation() << statPoint.x, statPoint.y, statPoint.z;
    transform.rotate(Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZ>::Ptr basePoint(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p;
    p.x = length;p.y = 0;p.z = 0;basePoint->points.push_back(p);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPoint (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::transformPointCloud (*basePoint, *transformedPoint, transform); //点云变换，矩形框最终落点

    return transformedPoint->points[0];
}


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile(argv[1], *source);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("testin数据点云可视化"));
	viewer->removeAllPointClouds();

    float pi = 3.1415926535898;

    float px = -1.8379112979037;
    float py = 50.807456665169;
    float pz = -0.19436326954409;
    float sx = 0.71917100215554;
    float sy = 0.8545055808636;
    float sz = 1.8646786162816;

    float yaw_before = 2.0910599303594;
    float yaw_after = -2.6213290500252895;


    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source, "sample cloud1");
    viewer->addCoordinateSystem(1.0);

    Eigen::AngleAxisf rotation_vector(yaw_before, Eigen::Vector3f(0, 0, 1));
    viewer->addCube(Eigen::Vector3f(px, py, pz), Eigen::Quaternionf(rotation_vector), sx, sy, sz);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0);
    
    pcl::PointXYZ st1;st1.x = px;st1.y = py;st1.z = pz;
    viewer->addArrow(getEndPoint(st1, yaw, sy), st1, 255, 0, 0, false, "arrow");


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    };

    return 0;
}