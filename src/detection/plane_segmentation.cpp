#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;

double THRESHOLD;

//平面検出関数
void planeDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::ModelCoefficients::Ptr coefficients,
                 pcl::PointIndices::Ptr inliers,
                 double threshold) 
{
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    //必須
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);//モデル
    seg.setMethodType(pcl::SAC_RANSAC);//検出手法
    seg.setDistanceThreshold(threshold); //閾値 0.5とか
    seg.segment(*inliers, *coefficients);
}

//平面除去関数
void planeRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                  pcl::PointIndices::Ptr inliers,
                  bool negative)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(negative);// true にすると平面を除去、false にすると平面以外を除去
    extract.filter(*cloud);
}

void callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    planeDetect(cloud, coefficients, inliers, THRESHOLD);
    planeRemoval(cloud, inliers, false);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*cloud, pc2); 
    pc2.header.frame_id = msg->header.frame_id;
    pc2.header.stamp = ros::Time::now();

    pub.publish(pc2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/cloud", 10, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/rm_plane", 10);

    nh.param<double>("threshold", THRESHOLD, 0.5);

    ros::spin();

    return 0;
}
