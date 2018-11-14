/*

transform cluster header frame_id from lidar to camera

Publish
    pub : cluster(lidar frame)

Subscribe
    sub : cluster(camera frame)

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
        tf::Transform transform;
        ros::Publisher pub;
        ros::Publisher pub_cloud;
        ros::Subscriber sub;
        ros::Subscriber sub_tf;
        string target_frame;
        string source_frame;
        bool flag;
    public:
        ClusterTransform();
        void Callback(const ObjectInfoArrayConstPtr msg);
        void tfCallback(const geometry_msgs::TransformConstPtr msg);
        void transform_pointcloud(CloudAPtr cloud, 
                                  CloudAPtr& trans_cloud);
        void cluster_transform(ObjectInfoArray object_array, 
                               vector<Clusters>& cluster_array);
        void getClusterInfo(CloudA pt, Cluster& cluster);
};

ClusterTransform::ClusterTransform()
    : nh("~")
{
    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);
    sub         = nh.subscribe("/objectinfo", 10, &ClusterTransform::Callback, this);
    sub_tf      = nh.subscribe("/transform",  10, &ClusterTransform::tfCallback, this);
    pub         = nh.advertise<ObjectInfoArray>("/objectinfo/tf", 10);
    pub_cloud   = nh.advertise<PointCloud2>("/cloud_vis", 10); 
    flag = false;
}

void ClusterTransform::tfCallback(const geometry_msgs::TransformConstPtr msg)
{
    tf::transformMsgToTF(*msg, transform);
    flag = true;
}

void ClusterTransform::transform_pointcloud(CloudAPtr cloud, 
                                            CloudAPtr& trans_cloud)
{
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll ,pitch, yaw);

    // cout<<setprecision(3)<<"----"<<target_frame<<"-->"<<source_frame
    //      <<" x:"<<x<<" y:"<<y<<" z:"<<z<<" roll:"<<roll<<" pitch:"<<pitch<<" yaw:"<<yaw<<endl;

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    // trans_cloud->header.frame_id = target_frame;
}


void ClusterTransform::Callback(const ObjectInfoArrayConstPtr msg)
{
    // transform listener
    if(!flag){
        std::cout<<"Wait Transform"<<std::endl;
        return;
    }
    
    std::cout<<"Callback"<<endl;
    
    vector<Clusters> cluster_array;
    cluster_transform(*msg, cluster_array);
    
    // visualize pointcloud
    CloudA cloud_vis;
    PointCloud2 pc2_vis;

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

        printf("----Cluster %2d x:%6.2f y:%6.2f z:%6.2f depth:%6.2f width:%6.2f height:%6.2f\n",
                int(i), cluster_array[i].data.x, cluster_array[i].data.y, cluster_array[i].data.z,
                cluster_array[i].data.depth, cluster_array[i].data.width, cluster_array[i].data.height);

        cloud_vis += cluster_array[i].points;
    }
    pub.publish(object_array);

    pcl::toROSMsg(cloud_vis, pc2_vis);
    pc2_vis.header.frame_id = target_frame;
    pc2_vis.header.stamp = ros::Time::now();
    pub_cloud.publish(pc2_vis);
}

void ClusterTransform::cluster_transform(ObjectInfoArray object_array, vector<Clusters>& cluster_array)
{
    for(size_t i=0;i<object_array.object_array.size();i++)
    {
        // Transform PointCloud from source_frame(lidar) to target_frame(camera)
        CloudAPtr cloud_in(new CloudA);
        CloudAPtr cloud_tf(new CloudA);
        pcl::fromROSMsg(object_array.object_array[i].points, *cloud_in);
        transform_pointcloud(cloud_in, cloud_tf);

        // get Cluster data
        Cluster data;
        getClusterInfo(*cloud_tf, data);

        // get cluster centroid
        PointA centroid;
        centroid.x = data.x;
        centroid.y = data.y;
        centroid.z = data.z;

        // Input clusters info
        Clusters cluster;
        cluster.centroid.points.push_back(centroid);
        cluster.points = *cloud_tf;
        cluster.data = data;
        cluster.Class = object_array.object_array[i].Class;

        cluster_array.push_back(cluster);
    }
}

void ClusterTransform::getClusterInfo(CloudA pt, Cluster& cluster)
{
    Vector3f centroid;
    centroid[0]=pt.points[0].x;
    centroid[1]=pt.points[0].y;
    centroid[2]=pt.points[0].z;

    Vector3f min_p;
    min_p[0]=pt.points[0].x;
    min_p[1]=pt.points[0].y;
    min_p[2]=pt.points[0].z;

    Vector3f max_p;
    max_p[0]=pt.points[0].x;
    max_p[1]=pt.points[0].y;
    max_p[2]=pt.points[0].z;

    for(size_t i=1;i<pt.points.size();i++){
        centroid[0]+=pt.points[i].x;
        centroid[1]+=pt.points[i].y;
        centroid[2]+=pt.points[i].z;
        if (pt.points[i].x<min_p[0]) min_p[0]=pt.points[i].x;
        if (pt.points[i].y<min_p[1]) min_p[1]=pt.points[i].y;
        if (pt.points[i].z<min_p[2]) min_p[2]=pt.points[i].z;

        if (pt.points[i].x>max_p[0]) max_p[0]=pt.points[i].x;
        if (pt.points[i].y>max_p[1]) max_p[1]=pt.points[i].y;
        if (pt.points[i].z>max_p[2]) max_p[2]=pt.points[i].z;
    }

    cluster.x=centroid[0]/(float)pt.points.size();
    cluster.y=centroid[1]/(float)pt.points.size();
    cluster.z=centroid[2]/(float)pt.points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_transform");

    ClusterTransform tf;

    ros::spin();
 
    return 0;
}
