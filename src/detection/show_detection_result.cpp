/* 
 * object detectionのみとtrackingを導入した場合の物体検出結果を比較する
 *
 * Publish:sensor_msgs::PointCloud2 物体の重心点
 * 
 * Subscribe:amsl_recog_msgs::ObjectInfoArray 物体の情報もろもろ
 */

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<amsl_recog_msgs/ObjectInfoWithROI.h>
#include<amsl_recog_msgs/ObjectInfoArray.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_ros/transforms.h>

class Show{
    private:
        ros::NodeHandle nh;

        ros::Publisher pub_detection;
        ros::Publisher pub_tracking;

        ros::Subscriber sub_detection;
        ros::Subscriber sub_tracking;

        void detection_callback(const amsl_recog_msgs::ObjectInfoArray::ConstPtr&);
        void tracking_callback(const amsl_recog_msgs::ObjectInfoArray::ConstPtr&);
    
    public:
        Show();
};

Show::Show()
    : nh("~")
{
    pub_detection = nh.advertise<sensor_msgs::PointCloud2>("/detection_points/compare", 1);
    pub_tracking = nh.advertise<sensor_msgs::PointCloud2>("/tracking_points/compare", 1);

    sub_detection = nh.subscribe("/iou", 10, &Show::detection_callback, this);
    sub_tracking  = nh.subscribe("/integrate", 10, &Show::tracking_callback, this);
}

void Show::detection_callback(const amsl_recog_msgs::ObjectInfoArray::ConstPtr& msg)
{
    std::cout<<"detection"<<std::endl;
    std::cout<<"frame:"<<msg->header.frame_id<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> position;

    for(size_t i=0;i<msg->object_array.size();i++){
        std::cout<<"Class:"<<msg->object_array[i].Class<<" "
                 <<msg->object_array[i].pose.position.x<<" "
                 <<msg->object_array[i].pose.position.y<<" "
                 <<msg->object_array[i].pose.position.z<<std::endl;
        pcl::PointXYZ p;
        p.x = msg->object_array[i].pose.position.x;
        p.y = msg->object_array[i].pose.position.y;
        p.z = msg->object_array[i].pose.position.z;
        position.push_back(p);
    }
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(position, pc2);
    pc2.header.frame_id = msg->header.frame_id;
    pc2.header.stamp = msg->header.stamp;

    pub_detection.publish(pc2);

}

void Show::tracking_callback(const amsl_recog_msgs::ObjectInfoArray::ConstPtr& msg)
{
    std::cout<<"tracking"<<std::endl;
    std::cout<<"frame:"<<msg->header.frame_id<<std::endl;
   
    pcl::PointCloud<pcl::PointXYZ> position;

    for(size_t i=0;i<msg->object_array.size();i++){
        std::cout<<"Class:"<<msg->object_array[i].Class<<" "
                 <<msg->object_array[i].pose.position.x<<" "
                 <<msg->object_array[i].pose.position.y<<" "
                 <<msg->object_array[i].pose.position.z<<std::endl;
        pcl::PointXYZ p;
        p.x = msg->object_array[i].pose.position.x;
        p.y = msg->object_array[i].pose.position.y;
        p.z = msg->object_array[i].pose.position.z;
        position.push_back(p);
    }
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(position, pc2);
    pc2.header.frame_id = msg->header.frame_id;
    pc2.header.stamp = msg->header.stamp;

    pub_tracking.publish(pc2);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "show_detection_result");

    ros::NodeHandle nh("~");

    Show sh;

    ros::spin();

    return 0;
}
