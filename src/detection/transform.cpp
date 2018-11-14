/*

transform cluster header frame_id from lidar to camera

Publish
    pub : cluster(target frame)

Subscribe
    sub : cluster(source_frame frame)

author
    Yudai Sadakuni
*/

#include <ros/ros.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace amsl_recog_msgs;

typedef pcl::PointXYZ PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Vector3f min_p;
    Vector3f max_p;
};

struct Clusters{
    Cluster data;
    CloudA centroid;
    CloudA points;
    string Class;
};

class ClusterTransform{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        string target_frame;
        string source_frame;
        bool flag;
        
        //transform
        tf::TransformListener listener;
        tf::StampedTransform  transform;

    public:
        ClusterTransform();
        void Callback(const ObjectInfoArrayConstPtr msg);
        void tf_listener();
        void cluster_transform(ObjectInfoArray object_array, 
                               vector<Clusters>& cluster_array);
};

ClusterTransform::ClusterTransform()
    : nh("~")
{
    nh.param<string>("target_frame", target_frame, "target_frame");
    nh.param<string>("source_frame", source_frame, "source_frame");
    sub = nh.subscribe("/objectinfo", 10, &ClusterTransform::Callback, this);
    pub = nh.advertise<ObjectInfoArray>("/objectinfo/tf", 10);
    flag = false;
}

void ClusterTransform::tf_listener()
{
    // tflistener
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, now, ros::Duration(1.0));
        listener.lookupTransform(target_frame, source_frame,  now, transform);
        flag = true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        flag = false;
    }
}

void ClusterTransform::Callback(const ObjectInfoArrayConstPtr msg)
{
    // transform listener
    tf_listener();
    if(!flag){
        std::cout<<"Wait Transform"<<std::endl;
        return;
    }
    
    // std::cout<<"Callback"<<endl;
    
    vector<Clusters> cluster_array;
    cluster_transform(*msg, cluster_array);
       
    // Object Info
    ObjectInfoArray object_array;
    ros::Time t = ros::Time::now();
    object_array.header.frame_id = target_frame;
    object_array.header.stamp = t;
    std::cout<<"cluster size:"<<cluster_array.size()<<endl;
    for(size_t i=0;i<cluster_array.size();i++)
    {
        PointCloud2 pc2_cloud;
        toROSMsg(cluster_array[i].points, pc2_cloud);
        pc2_cloud.header.frame_id = target_frame;
        pc2_cloud.header.stamp = t;

        amsl_recog_msgs::ObjectInfoWithROI data;
        data.header.frame_id    = target_frame;
        data.header.stamp = t;
        data.Class = cluster_array[i].Class;
        data.pose.position.x = cluster_array[i].data.x;
        data.pose.position.y = cluster_array[i].data.y;
        data.pose.position.z = cluster_array[i].data.z;
        data.pose.orientation.x = 0;
        data.pose.orientation.y = 0;
        data.pose.orientation.z = 0;
        data.pose.orientation.w = 1;
        data.width  = cluster_array[i].data.width;
        data.height = cluster_array[i].data.height;
        data.depth  = cluster_array[i].data.depth;
        data.points = pc2_cloud;
        object_array.object_array.push_back(data);

        printf("----ID:%2d Class:%s x:%6.2f y:%6.2f z:%6.2f depth:%6.2f width:%6.2f height:%6.2f\n\n",
                int(i), cluster_array[i].Class.c_str(),
                cluster_array[i].data.x, cluster_array[i].data.y, cluster_array[i].data.z,
                cluster_array[i].data.depth, cluster_array[i].data.width, cluster_array[i].data.height);
    }
    pub.publish(object_array);
}

void ClusterTransform::cluster_transform(ObjectInfoArray object_array, vector<Clusters>& cluster_array)
{
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll ,pitch, yaw);
    // std::cout<<setprecision(3)<<"----"<<target_frame<<"-->"<<source_frame
    //    <<" x:"<<x<<" y:"<<y<<" z:"<<z<<" roll:"<<roll<<" pitch:"<<pitch<<" yaw:"<<yaw<<std::endl;
    Eigen::Translation3f trans(x, y, z);
    Eigen::AngleAxisf rotation_x(roll,  Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotation_y(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_z(yaw,   Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotation = (rotation_x * rotation_y * rotation_z).matrix();
    Eigen::Affine3f affine = trans * rotation;

    for(size_t i=0;i<object_array.object_array.size();i++)
    {
        {{{
        /*
        std::cout<<object_array.object_array[i].pose<<std::endl;
        pcl::PointXYZ p;
        p.x = object_array.object_array[i].pose.position.x;
        p.y = object_array.object_array[i].pose.position.y;
        p.z = object_array.object_array[i].pose.position.z;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> trans_cloud;
        cloud.push_back(p);
        tf::Transform tf;
        tf.setOrigin(transform.getOrigin());
        tf.setRotation(transform.getRotation());
        pcl_ros::transformPointCloud(cloud, trans_cloud, tf);
        for(size_t i=0;i<trans_cloud.points.size();i++)
            std::cout<<"x:"<<trans_cloud.points[i].x<<" "
                     <<"y:"<<trans_cloud.points[i].y<<" "
                     <<"z:"<<trans_cloud.points[i].z<<std::endl;
        */
        }}}
        Eigen::Vector3f pose(object_array.object_array[i].pose.position.x,
                             object_array.object_array[i].pose.position.y,
                             object_array.object_array[i].pose.position.z);

        Eigen::Vector3f trans_pose = affine * pose;

        std::cout<<"pose:"<<pose<<std::endl;
        std::cout<<"trans:"<<trans_pose<<std::endl;


        // get cluster centroid;
        PointA centroid;
        centroid.x = trans_pose(0); centroid.y = trans_pose(1); centroid.z = trans_pose(2);
        
        // set cluster data
        Cluster cluster;
        cluster.x = trans_pose(0); cluster.y = trans_pose(1); cluster.z = trans_pose(2);
        cluster.height = object_array.object_array[i].height;
        cluster.width  = object_array.object_array[i].width;
        cluster.depth  = object_array.object_array[i].depth;

        // set clusters info
        Clusters clusters;
        clusters.data = cluster;
        clusters.centroid.points.push_back(centroid);
        clusters.Class = object_array.object_array[i].Class;

        // transform pointcloud from source_frame to target_frame
        CloudAPtr cloud_in(new CloudA);
        CloudAPtr cloud_tf(new CloudA);
        pcl::fromROSMsg(object_array.object_array[i].points, *cloud_in);
        tf::Transform tf;
        tf.setOrigin(transform.getOrigin());
        tf.setRotation(transform.getRotation());
        pcl_ros::transformPointCloud(*cloud_in, *cloud_tf, tf);
        clusters.points = *cloud_tf;

        cluster_array.push_back(clusters);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_transform");

    ClusterTransform tf;

    ros::spin();
 
    return 0;
}
