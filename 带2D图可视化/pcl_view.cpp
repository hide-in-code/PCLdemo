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

#include "cJSON.h"
#include "YcPCLVisualizer.h"

pcl::PointXYZ getEndPoint(pcl::PointXYZ statPoint, float yaw, float length)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 
    transform.translation() << statPoint.x, statPoint.y, statPoint.z;
    transform.rotate(Eigen::AngleAxisf (yaw, Eigen::Vector3f::UnitZ()));

    // std::cout << yaw << std::endl;
    // std::cout << yaw << std::endl;
    // std::cout << transform.matrix() << std::endl;
    // std::cout << "======================" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr basePoint(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p;
    p.x = length; p.y = 0; p.z = 0; basePoint->points.push_back(p);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPoint (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::transformPointCloud (*basePoint, *transformedPoint, transform); //点云变换，矩形框最终落点

    return transformedPoint->points[0];
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile(argv[1], *source);
    boost::shared_ptr<YcPCLVisualizer> viewer(new YcPCLVisualizer("viewer", true));

    std::string jsonPath = argv[2];
    const char* Jfilename = jsonPath.data();

    //显示图片
    std::string logofilename = argv[3];
	double x = 0, y = 0.79, x_wide = 0.4, y_wide = 0.2, opacity = 1;
	viewer->addLogoWidgetToview(viewer->getRenderWindow()->GetInteractor(), logofilename, x, y, x_wide, y_wide, opacity);

    viewer->removeAllPointClouds();//清空点云
	viewer->addPointCloud(source);//加载点云

    FILE *f;
    long len;
    char *data;

    f = fopen(Jfilename, "rb");
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    data = (char *)malloc(len + 1);
    fread(data, 1, len, f);
    fclose(f);

    char *out;
    cJSON *json;
    json = cJSON_Parse(data);

    cJSON *arrayItem = cJSON_GetObjectItem(json, "marks");
    int arr_size = cJSON_GetArraySize(arrayItem);

    cJSON *object;
    cJSON *scale, *sx, *sy, *sz;
    cJSON *position, *px, *py, *pz;
    cJSON *rotation, *rx, *ry, *rz;
    cJSON *yaw;

    for (int boxid = 0; boxid < arr_size; boxid ++) {
        object = cJSON_GetArrayItem(arrayItem, boxid);

        position = cJSON_GetObjectItem(object, "position");
        px = cJSON_GetObjectItem(position, "x");
        py = cJSON_GetObjectItem(position, "y");
        pz = cJSON_GetObjectItem(position, "z");

        scale = cJSON_GetObjectItem(object, "scale");
        sx = cJSON_GetObjectItem(scale, "x");
        sy = cJSON_GetObjectItem(scale, "y");
        sz = cJSON_GetObjectItem(scale, "z");

        rotation = cJSON_GetObjectItem(object, "rotation");
        rx = cJSON_GetObjectItem(rotation, "x");
        ry = cJSON_GetObjectItem(rotation, "y");
        rz = cJSON_GetObjectItem(rotation, "z");
        yaw = rz;

        //绘制
        Eigen::AngleAxisf rotation_vector(yaw->valuedouble, Eigen::Vector3f(0, 0, 1));
        viewer->addCube(Eigen::Vector3f(px->valuedouble, py->valuedouble, pz->valuedouble), Eigen::Quaternionf(rotation_vector), sx->valuedouble, sy->valuedouble, sz->valuedouble, std::to_string(boxid));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(boxid));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, std::to_string(boxid));

        //箭头
        pcl::PointXYZ st;
        st.x = px->valuedouble;
        st.y = py->valuedouble;
        st.z = pz->valuedouble;
        viewer->addArrow(getEndPoint(st, yaw->valuedouble, sy->valuedouble * 2), st, 255, 0, 0, false, "arrow_" + std::to_string(boxid));
    }

    viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
	viewer->setCameraPosition(0, 0, 2, 0, -1, 0, 0);
    while(!viewer->wasStopped())
    {
	   viewer->spinOnce(100);
    }

    return (0);
}