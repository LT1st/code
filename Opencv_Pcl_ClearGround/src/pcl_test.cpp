#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp" // Include short list of convenience functions for rendering
//#include <librealsense2/example.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>                     //点云（类型）
#include <pcl/visualization/cloud_viewer.h>      //点云查看窗口头文件
#include <pcl/ModelCoefficients.h>               //模型系数 Used when add or delet or device the PC
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法  随机样本一致性算法	方法类型
#include <pcl/sample_consensus/model_types.h>    //模型定义头文件     随机样本一致性算法	模型类型
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>              //Updated after the artical written
#include <librealsense2/rs.hpp>                 // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>                   // Include OpenCV API
#include <iostream>
#include <stdio.h>
#include <pcl/io/io.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/pcl_visualizer.h>   //visuailization methord 最强大的是PCLvisualizer
#include <pcl/visualization/range_image_visualizer.h> //深度图可视化
#include <pcl/console/parse.h>                  //命令行参数解析
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>


int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    char file_name[] = "realsensPic1.ply";
    if (pcl::io::loadPLYFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
    {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
    
    }
    //cerr常用于输出错误信息与其他不属于正常逻辑的输出内容
    std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//存储输出的模型的系数
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//存储内点，使用的点
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选设置
    seg.setOptimizeCoefficients (true);
    // 必须设置
    seg.setModelType (pcl::SACMODEL_PLANE); //设置模型类型，检测平面
    seg.setMethodType (pcl::SAC_RANSAC);    //设置方法【聚类或随机样本一致性】
    seg.setDistanceThreshold (0.02);        //The Thereshould of segment
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);  //分割操作

    if (inliers->indices.size() == 0)       //根据内点数量，判断是否成功
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    //显示模型的系数
     std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                         << coefficients->values[1] << " "
                                         << coefficients->values[2] << " " 
                                         << coefficients->values[3] << std::endl;

    // 提取地面
    //通过分割算法提取部分点云数据子集的下标索引
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered);
    // 提取除地面外的物体 提取相反的点云集剩余点云
    extract.setNegative (true);
    extract.filter (*cloud_filtered); 

    // std::cerr << "Ground cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;
    // //打印出平面模型
    // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    //  << coefficients->values[1] << " "
    //  << coefficients->values[2] << " "
    //  << coefficients->values[3] << std::endl;
    //显示估计平面模型过程中使用的内点
    // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    // for (size_t i = 0; i < inliers->indices.size(); ++i)
    //  std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    //  << cloud->points[inliers->indices[i]].y << " "
    //  << cloud->points[inliers->indices[i]].z << std::endl;

    // 点云可视化
    pcl::visualization::CloudViewer viewer("Filtered");
    viewer.showCloud(cloud_filtered);

    while(!viewer.wasStopped()){
    }
    return (0);
}