#include <ros/ros.h>
#include <amsl_recog_msgs/ObjectInfoWith.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

amsl_recog_msgs::ObjectInfoArray cluster_array;
amsl_recog_msgs::ObjectInfoArray tracker_array;

class Integrater{
pricate:
    ros::NodeHandle nh;
    ros::Subscriber sub_cluster;
    ros::Subscriber sub_tracker;
    ros::Publisher pub;

    amsl_recog_msgs::ObjectInfoArray tracker;
    amsl_recog_msgs::ObjectInfoArray cluster;
    
    bool tracker_flag;
    double threshold;
        
    public:
        Integrater();
        void cluster_callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg);
        void tracker_callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr& msg);
        void integration();
};

Integrater::Integrater()
    : nh("~")
{
    sub_cluster = nh.subscribe("/cluster", 10, &Integrater::cluster_callback, this);
    sub_tracker = nh.subscribe("/tracker", 10, &Integrater::tracker_callback, this);
    pub = nh.advertise<amsl_recog_msgs::ObjectInfoArray>("/integration". 10);

    nh.param<double>("threshold", threshold, 0.20);

    tracker_flag = false;


}

void Integrater::cluster_callback(const amsl_recog_msgs::ObjectInfoArrayConst::Ptr msg)
{
    printf("cluster callback\n");
    tracker = *msg;

    if(!tracker_flag)
        pub.publish(*msg);
    else
        integration();
}

void Integrater::tracker_callback(const amsl_recog_msgs::ObjectInfoArrayConst::Ptr msg)
{
    printf("tracker callback\n");
    tracker_flag = true;
    tracker = *msg;
}

void Integrater::integration()
{
    amsl_recog_msgs::ObjectInfoArray integrate;

    integrate = cluster;

    double matrix[int(tracker.object_array.size())][int(cluster.object_array.size()];

    for(size_t i=0;i<tracker.object_array.size();i++)
    {
        double min_distance = INFINITY;
        double ID = 0;
        for(size_t j=0;i<cluster.object_array.size();j++)
        {
            double distance = sqrt(pow(tracker.object_array[i].pose.position.x-cluster.object_array[j].pose.position.x, 2)+
                                   pow(tracker.object_array[i].pose.position.y-cluster.object_array[j].pose.position.y, 2));





int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_integrater");

    Integrater in;

    ros::spin();

    return 0;
}
