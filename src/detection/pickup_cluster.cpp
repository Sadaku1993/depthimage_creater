/*

PickUp Cluster ( Human and Car)

Publish
    pub_bbox        : jsk_visualize
    pub_cluster     : cluster
Subscribe
    sub : amsl_recog_msgs

Causion:
    using original msg(amsl_recog_msgs, jsk_recognition_msgs)

author:
    Yudai Sadakuni

*/

# include <ros/ros.h>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>

# include <jsk_recognition_msgs/BoundingBox.h>
# include <jsk_recognition_msgs/BoundingBoxArray.h>
# include <amsl_recog_msgs/ObjectInfoWithROI.h>
# include <amsl_recog_msgs/ObjectInfoArray.h>

using namespace std;

class PickUp{
    public:
        PickUp();

    private:
        ros::NodeHandle nh;

        ros::Publisher pub_bbox;
        ros::Publisher pub_cluster;

        ros::Subscriber sub_cluster;

        void cluster_callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg);

        double max_width, max_height, max_depth;
        double min_width, min_height, min_depth;

};

PickUp::PickUp()
    : nh("~")
{
    sub_cluster = nh.subscribe("/cluster/objectinfo", 10, &PickUp::cluster_callback, this);

    pub_cluster  = nh.advertise<amsl_recog_msgs::ObjectInfoArray>("/cluster/objectinfo/pickup", 10);
    pub_bbox     = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/cluster/bbox/pickup", 10);

    nh.param<double>("max_width" , max_width,  1.5);
    nh.param<double>("max_height", max_height, 2.0);
    nh.param<double>("max_depth",  max_depth,  1.5);
    nh.param<double>("min_width" , min_width,  0.15);
    nh.param<double>("min_height", min_height, 0.8);
    nh.param<double>("min_depth",  min_depth,  0.15);
}

void PickUp::cluster_callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg)
{
    std::cout<<"size:"<<msg->object_array.size()<<std::endl;

    ros::Time time = ros::Time::now();
    amsl_recog_msgs::ObjectInfoArray object_array;
    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    object_array.header.frame_id = msg->header.frame_id;
    // object_array.header.stamp = time;
    object_array.header.stamp = msg->header.stamp;

    bbox_array.header.frame_id = msg->header.frame_id;
    bbox_array.header.stamp = time;

    for(size_t i=0;i<msg->object_array.size();i++)
    {
        amsl_recog_msgs::ObjectInfoWithROI cluster = msg->object_array[i];

        if(min_width  < cluster.width   && cluster.width  < max_width &&
           min_height < cluster.height  && cluster.height < max_height &&
           min_depth  < cluster.depth   && cluster.depth  < max_depth)
        {
            amsl_recog_msgs::ObjectInfoWithROI data;
            data.header.frame_id = cluster.header.frame_id;
            data.header.stamp = time;
            data.pose   = cluster.pose;
            data.width  = cluster.width;
            data.height = cluster.height;
            data.depth  = cluster.depth;
            data.points = cluster.points;
            object_array.object_array.push_back(data);

            jsk_recognition_msgs::BoundingBox bbox;
            bbox.header.frame_id = cluster.header.frame_id;
            bbox.header.stamp = time;
            bbox.header.seq = 0;
            bbox.pose = cluster.pose;
            bbox.dimensions.x = cluster.depth;
            bbox.dimensions.y = cluster.width;
            bbox.dimensions.z = cluster.height;
            bbox_array.boxes.push_back(bbox);
        }
    }

    pub_cluster.publish(object_array);
    pub_bbox.publish(bbox_array);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pickup_cluster");

    PickUp pu;

    ros::spin();

    return 0;
}
