#include <ros/ros.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>
#include <amsl_recog_msgs/ObjectInfoWithROI.h>


void callback(const amsl_recog_msgs::ObjectInfoArray::ConstPtr &msg)
{
    std::cout<<"--------"<<std::endl;
    for(size_t i=0;i<msg->object_array.size();i++)
        std::cout<<"x: "<<msg->object_array[i].pose.position.x<<" "<<
                   "y: "<<msg->object_array[i].pose.position.y<<" "<<
                   "z: "<<msg->object_array[i].pose.position.z<<std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "showinfo");

    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/iou", 10, callback);

    ros::spin();

    return 0;
}
