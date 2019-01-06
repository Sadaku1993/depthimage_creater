#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>

#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

template<typename T_bfr, typename T_afr>
class NormalEstimation{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        double search_radius;
    public:
        NormalEstimation();
        void callback(const sensor_msgs::PointCloud2::ConstPtr&);
        void normal_estimation(typename pcl::PointCloud<T_bfr>::Ptr cloud,
                               typename pcl::PointCloud<T_afr>::Ptr cloud_normal);
        void main();
};

template<typename T_bfr, typename T_afr>
NormalEstimation<T_bfr, T_afr>::NormalEstimation()
    : nh("~")
{
    sub = nh.subscribe("/cloud", 10, &NormalEstimation::callback, this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_normal", 10);
    nh.getParam("search_radius", search_radius);
}

template<typename T_bfr, typename T_afr>
void NormalEstimation<T_bfr, T_afr>::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::cout<<"callback"<<std::endl;
    typename pcl::PointCloud<T_bfr>::Ptr cloud(new pcl::PointCloud<T_bfr>);
    pcl::fromROSMsg(*msg, *cloud);

    std::cout<<"normal estimation"<<std::endl;
    typename pcl::PointCloud<T_afr>::Ptr cloud_normal(new pcl::PointCloud<T_afr>);
    normal_estimation(cloud, cloud_normal);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*cloud_normal, pc2);
    pc2.header.frame_id = msg->header.frame_id;
    pc2.header.stamp = msg->header.stamp;
    pub.publish(pc2);
}

template<typename T_bfr, typename T_afr>
void NormalEstimation<T_bfr, T_afr>::normal_estimation(typename pcl::PointCloud<T_bfr>::Ptr cloud,
                                                       typename pcl::PointCloud<T_afr>::Ptr cloud_normal)
{
    pcl::NormalEstimationOMP<T_bfr, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    typename pcl::search::KdTree<T_bfr>::Ptr tree (new pcl::search::KdTree<T_bfr> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (search_radius);
    ne.compute(*normals);

    T_afr tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		if(!std::isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!std::isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		cloud_normal->points.push_back(tmp);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_online");

    NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;

    ros::spin();

    return 0;
}
