#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg)
{
    sensor_msgs::PointCloud2 pc2;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (size_t i=0;i<msg->object_array.size();i++)
    {
        amsl_recog_msgs::ObjectInfoWithROI cluster = msg->object_array[i];
        pcl::PointXYZ p;
        p.x = cluster.pose.position.x;
        p.y = cluster.pose.position.y;
        p.z = cluster.pose.position.z;
        cloud.push_back(p);
    }

    pcl::toROSMsg(cloud, pc2);
    pc2.header.frame_id = msg->header.frame_id;
    pc2.header.stamp = ros::Time::now();
    pub.publish(pc2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_position");

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/objectinfo", 10, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster_position", 10);

    ros::spin();

    return 0;
}

