#include <ros/ros.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

ros::Publisher pub_bbox;

void boundingbox(const amsl_recog_msgs::ObjectInfoArray msg,
                ros::Publisher pub)
{
    jsk_recognition_msgs::BoundingBoxArray bbox_array;
    bbox_array.header.frame_id = msg.header.frame_id;
    bbox_array.header.stamp = ros::Time::now();

    for(int i=0;i<int(msg.object_array.size());i++){
        amsl_recog_msgs::ObjectInfoWithROI roi = msg.object_array[i];
        jsk_recognition_msgs::BoundingBox bbox;
        bbox.header.frame_id = msg.header.frame_id;
        bbox.header.stamp    = ros::Time::now();
        bbox.pose            = roi.pose;
        bbox.dimensions.x    = roi.depth;
        bbox.dimensions.y    = roi.width;
        bbox.dimensions.z    = roi.height;
        if(roi.Class=="person")
            bbox.label = 1;
        else
            bbox.label = 0;
        bbox_array.boxes.push_back(bbox);
    }
    pub.publish(bbox_array);
}

void callback(const amsl_recog_msgs::ObjectInfoArrayConstPtr msg)
{
    boundingbox(*msg, pub_bbox);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_boundingbox");

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/objectinfo", 10, callback);

    pub_bbox = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bbox", 10);

    ros::spin();

    return 0;
}
