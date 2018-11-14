/*

author : Yudai Sadakuni

split pointcloud

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace sensor_msgs;

ros::Publisher pub_low;
ros::Publisher pub_high;

double threshold;

void callback(const PointCloud2ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZI> cloud_low;
    pcl::PointCloud<pcl::PointXYZI> cloud_high;

    for(size_t i=0;i<cloud->points.size();i++)
    {
        if(cloud->points[i].z < threshold)
            cloud_low.points.push_back(cloud->points[i]);
        else
            cloud_high.points.push_back(cloud->points[i]);
    }

    PointCloud2 pc2_low;
    toROSMsg(cloud_low, pc2_low);
    pc2_low.header.frame_id = msg->header.frame_id;
    pc2_low.header.stamp = ros::Time::now();
    pub_low.publish(pc2_low);

    PointCloud2 pc2_high;
    toROSMsg(cloud_high, pc2_high);
    pc2_high.header.frame_id = msg->header.frame_id;
    pc2_high.header.stamp = ros::Time::now();
    pub_high.publish(pc2_high);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "split_pointcloud");
    ros::NodeHandle nh("~");

    nh.getParam("threshold", threshold);

    ros::Subscriber sub = nh.subscribe("/cloud", 10, callback);

    pub_low  = nh.advertise<PointCloud2>("/cloud/low", 10);
    pub_high = nh.advertise<PointCloud2>("/cloud/high", 10);

    ros::spin();

    return 0;
}
