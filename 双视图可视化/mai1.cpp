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
    p.x = length;p.y = 0;;p.z = 0;basePoint->points.push_back(p);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPoint (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::transformPointCloud (*basePoint, *transformedPoint, transform); //点云变换，矩形框最终落点

    return transformedPoint->points[0];
}


int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr source2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile(argv[1], *source);
	pcl::io::loadPCDFile(argv[2], *source2);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("testin数据点云可视化"));
	viewer->removeAllPointClouds();

    float pi = 3.1415926535898;


    // "position": {
    //     "x": 60.612617426796,
    //     "y": -15.971879734291,
    //     "z": -0.45999269349637
    // },
    // "rotation": {
    //     "x": 0,
    //     "y": 0,
    //     "z": -0.40698402099513
    // },
    // "size": {
    //     "x": 4.0119524957884,
    //     "y": 2.0293994015133,
    //     "z": 1.5472277277614
    // },


    float px = 60.612617426796;
    float py = -15.971879734291;
    float pz = -0.45999269349637;
    float sx = 4.0119524957884;
    float sy = 2.0293994015133;
    float sz = 1.5472277277614;
    float yaw = 0.40698402099513;

    float px2 = 60.612617426796;
    float py2 = -15.971879734291;
    float pz2 = -0.45999269349637;
    float sx2 = 4.0119524957884;
    float sy2 = 2.0293994015133;
    float sz2 = 1.5472277277614;
    float yaw2 = 0.40698402099513;







    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addPointCloud<pcl::PointXYZ>(source, "sample cloud1", v1);

    viewer->addCoordinateSystem(1.0);

    Eigen::AngleAxisf rotation_vector(yaw, Eigen::Vector3f(0, 0, 1));
    viewer->addCube(Eigen::Vector3f(px, py, pz), Eigen::Quaternionf(rotation_vector), sx, sy, sz, "1", v1);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "1");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "1");
    pcl::PointXYZ st1;st1.x = px;st1.y = py;st1.z = pz;
    viewer->addArrow(getEndPoint(st1, yaw, sy), st1, 255, 0, 0, false, "arrow", v1);




    // Eigen::AngleAxisf rotation_vector3(yaw3, Eigen::Vector3f(0, 0, 1));
    // viewer->addCube(Eigen::Vector3f(px3, py3, pz3), Eigen::Quaternionf(rotation_vector3), sx3, sy3, sz3, "3", v1);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "3");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "3");
    // pcl::PointXYZ st3;st3.x = px3;st3.y = py3;st3.z = pz3;
    // viewer->addArrow(getEndPoint(st3, yaw3, sy3), st3, 255, 0, 0, false, "arrow3", v1);


    int v2(2);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.2, 0.2, 0.2, v2);
    viewer->addPointCloud<pcl::PointXYZ>(source2, "sample cloud2", v2);

    viewer->addCoordinateSystem(1.0);

    Eigen::AngleAxisf rotation_vector2(yaw2, Eigen::Vector3f(0, 0, 1));
    viewer->addCube(Eigen::Vector3f(px2, py2, pz2), Eigen::Quaternionf(rotation_vector2), sx2, sy2, sz2, "2", v2);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "2");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "2");
    pcl::PointXYZ st2;st2.x = px2;st2.y = py2;st2.z = pz2;
    viewer->addArrow(getEndPoint(st2, yaw2, sy2), st2, 255, 0, 0, false, "arrow2", v2);

    // Eigen::AngleAxisf rotation_vector4(yaw4, Eigen::Vector3f(0, 0, 1));
    // viewer->addCube(Eigen::Vector3f(px4, py4, pz4), Eigen::Quaternionf(rotation_vector4), sx4, sy4, sz4, "4", v2);
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "4");
    // viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "4");
    // pcl::PointXYZ st4;st4.x = px4;st4.y = py4;st4.z = pz4;
    // viewer->addArrow(getEndPoint(st4, yaw4, sy4), st4, 255, 0, 0, false, "arrow4", v2);


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    };

    return 0;
}