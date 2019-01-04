#include <ros/ros.h>
#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class RemoveCluster{
    private:
        ros::NodeHandle nh;

        ros::Publisher pub;

        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        message_filters::Subscriber<amsl_recog_msgs::ObjectInfoArray> cluster_sub;
	    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, amsl_recog_msgs::ObjectInfoArray> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;

        void callback(const sensor_msgs::PointCloud2::ConstPtr&,
                      const amsl_recog_msgs::ObjectInfoArray::ConstPtr&);

        void rm_cluster(pcl::PointCloud<pcl::PointXYZ> cloud,
                        amsl_recog_msgs::ObjectInfoArray cluster,
                        pcl::PointCloud<pcl::PointXYZ>& output);

        int grid_dimentions = 100;
        float cell_size = 0.25;

    public:
        RemoveCluster();
};


RemoveCluster::RemoveCluster()
    : nh("~"),
      pc_sub(nh, "/cloud/lcl", 100),
      cluster_sub(nh, "/integrate", 100),
      sync(MySyncPolicy(100), pc_sub, cluster_sub)
{
    sync.registerCallback(boost::bind(&RemoveCluster::callback,this, _1, _2));
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cluster/human/removed", 1);
}


void RemoveCluster::callback(const sensor_msgs::PointCloud2::ConstPtr& pc2,
                             const amsl_recog_msgs::ObjectInfoArray::ConstPtr& cluster)
{
    std::cout<<"callback"<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> output;

    pcl::fromROSMsg(*pc2, cloud);

    rm_cluster(cloud, *cluster, output);

    sensor_msgs::PointCloud2 pc2_output;
    pcl::toROSMsg(output, pc2_output);
    pc2_output.header.frame_id = pc2->header.frame_id;
    pc2_output.header.stamp = pc2->header.stamp;
    // pc2_output.header.stamp = ros::Time::now();

    pub.publish(pc2_output);
}

void RemoveCluster::rm_cluster(pcl::PointCloud<pcl::PointXYZ> cloud,
                               amsl_recog_msgs::ObjectInfoArray cluster,
                               pcl::PointCloud<pcl::PointXYZ>& output)
{
    std::cout<<"remove"<<std::endl;

    amsl_recog_msgs::ObjectInfoArray person;
    for(size_t i=0;i<cluster.object_array.size();i++){
        if(cluster.object_array[i].Class == "person")
            person.object_array.push_back(cluster.object_array[i]);
    }

    bool init[grid_dimentions][grid_dimentions];
    memset(&init, false, grid_dimentions*grid_dimentions);

    for(size_t i=0;i<person.object_array.size();i++)
    {
        amsl_recog_msgs::ObjectInfoWithROI roi = person.object_array[i];

        int x = grid_dimentions/2 + roi.pose.position.x/cell_size;
        int y = grid_dimentions/2 + roi.pose.position.y/cell_size;

        int min_x = x - roi.depth*0.6/cell_size;
        int min_y = y - roi.width*0.6/cell_size;
        int max_x = x + roi.depth*0.6/cell_size;
        int max_y = y + roi.width*0.6/cell_size;

        for(int X=min_x; X<=max_x; X++){
            for(int Y=min_y; Y<=max_y; Y++){
                init[X][Y] = true;
            }
        }
    }

    for(size_t i=0;i<cloud.points.size();i++){
        int x = grid_dimentions/2 + cloud.points[i].x/cell_size;
        int y = grid_dimentions/2 + cloud.points[i].y/cell_size;

        if(!init[x][y])
            output.push_back(cloud.points[i]);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_cluster");
    std::cout<<"remove_cluster"<<std::endl;

    RemoveCluster rc;

    ros::spin();

    return 0;
}
