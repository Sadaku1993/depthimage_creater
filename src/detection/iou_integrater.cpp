#include <ros/ros.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>
#include <amsl_recog_msgs/ObjectInfoWithROI.h>

#include <sys/time.h>
using namespace amsl_recog_msgs;

ObjectInfoArray cluster_array;

bool iou0 = false;
bool iou1 = false;
bool iou2 = false;
bool iou3 = false;
bool iou4 = false;

void iou0_callback(const ObjectInfoArray::ConstPtr &msg)
{
    if(!iou0){
        for(size_t i=0;i<msg->object_array.size();i++)
            cluster_array.object_array.push_back(msg->object_array[i]);
        iou0 = true;
    }
}

void iou1_callback(const ObjectInfoArray::ConstPtr &msg)
{
    if(!iou1){
        for(size_t i=0;i<msg->object_array.size();i++)
            cluster_array.object_array.push_back(msg->object_array[i]);
        iou1 = true;
    }
}

void iou2_callback(const ObjectInfoArray::ConstPtr &msg)
{
    if(!iou2){
        for(size_t i=0;i<msg->object_array.size();i++)
            cluster_array.object_array.push_back(msg->object_array[i]);
        iou2 = true;
    }
}

void iou3_callback(const ObjectInfoArray::ConstPtr &msg)
{
    if(!iou3){
        for(size_t i=0;i<msg->object_array.size();i++)
            cluster_array.object_array.push_back(msg->object_array[i]);
        iou3 = true;
    }
}
void iou4_callback(const ObjectInfoArray::ConstPtr &msg)
{
    if(!iou4){
        for(size_t i=0;i<msg->object_array.size();i++)
            cluster_array.object_array.push_back(msg->object_array[i]);
        iou4 = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iou_integrater");

    ros::NodeHandle nh("~");

    ros::Subscriber sub_iou0 = nh.subscribe("/iou0", 10, iou0_callback);
    ros::Subscriber sub_iou1 = nh.subscribe("/iou1", 10, iou1_callback);
    ros::Subscriber sub_iou2 = nh.subscribe("/iou2", 10, iou2_callback);
    ros::Subscriber sub_iou3 = nh.subscribe("/iou3", 10, iou3_callback);
    ros::Subscriber sub_iou4 = nh.subscribe("/iou4", 10, iou4_callback);

    ros::Publisher pub = nh.advertise<ObjectInfoArray>("/iou", 10);

    ros::Rate rate(100);

    double threshold;
    std::string frame_id;

    nh.param<double>("threshold", threshold, 0.1);
    nh.param<std::string>("frame_id",  frame_id, "centerlaser");

    struct timeval s, e;
    gettimeofday(&s, NULL);
    
    while(ros::ok())
    {
        gettimeofday(&e, NULL);

        double timer = (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6;

        if(threshold<timer){
            cluster_array.header.frame_id = frame_id;
            cluster_array.header.stamp = ros::Time::now();
            pub.publish(cluster_array);
            std::cout<<"publish"<<std::endl;
            for(size_t i=0;i<cluster_array.object_array.size();i++){
                ObjectInfoWithROI data = cluster_array.object_array[i];
                std::cout<<"x:"<<data.pose.position.x<<" y:"<<data.pose.position.y<<" z:"<<data.pose.position.z<<std::endl;
            }
            cluster_array.object_array.clear();
            iou0 = false;
            iou1 = false;
            iou2 = false;
            iou3 = false;
            iou4 = false;
            gettimeofday(&s, NULL);
        }
    
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
