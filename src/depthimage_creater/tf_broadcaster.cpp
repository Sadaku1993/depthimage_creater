#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

class Listener{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;

        string parent_frame;
        string child_frame;

        tf::Transform transform;
        bool flag;

    public:
        Listener();
        void callback(const geometry_msgs::TransformConstPtr msg);
        void broadcaster();
};

Listener::Listener()
    : nh("~")
{
    nh.getParam("parent_frame", parent_frame);
    nh.getParam("child_frame", child_frame);

	sub = nh.subscribe("/transform", 10, &Listener::callback, this);

    flag = false;
}

void Listener::callback(const geometry_msgs::TransformConstPtr msg)
{
    tf::transformMsgToTF(*msg, transform);
    flag = true;
}

void Listener::broadcaster()
{
    if(flag){
        static tf::TransformBroadcaster br;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_broadcast");

    Listener lr;

    ros::Rate rate(20);

    while(ros::ok()){
        lr.broadcaster();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
