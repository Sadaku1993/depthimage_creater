/*
 * data_saver.cpp
 *
 * PointCloud, Image, tfを保存する
 *
 * 保存するタイミングは移動量や物体の有無などで判定(今は移動量のみ 20181020)
 * 保存するディレクトリは
 *    Image : /depthimage_creater/image
 *    tf    : /depthimage_creater/tf
 *    cloud : /depthimage_creater/cloud
 *
 * author:Yudai Sadakuni
 *
 */


#include <ros/ros.h>
#include <ros/package.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
class DataSaver{
    private:
        ros::NodeHandle nh;
        ros::Subscriber image_sub;
        ros::Subscriber cloud_sub;
        ros::Subscriber odom_sub;
        std::string target_frame;
        std::string source_frame;
        tf::TransformListener listener;
        tf::StampedTransform  transform;
        sensor_msgs::Image image;
        sensor_msgs::PointCloud2 pc2;

        nav_msgs::Odometry old_odom;
        double distance;
        bool odom_flag;
        double threshold;
        bool first_flag;

        std::string package_path;
        std::string device;
        int count;

    public:
        DataSaver();
        void imageCallback(const sensor_msgs::Image::ConstPtr&);
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr&);
        void odomCallback(const nav_msgs::Odometry::ConstPtr&);
        bool tf_listener();
        bool save_process();
};

DataSaver::DataSaver()
    : nh("~")
{
    image_sub = nh.subscribe("/image", 10, &DataSaver::imageCallback, this);
    cloud_sub = nh.subscribe("/cloud", 10, &DataSaver::cloudCallback, this);
    odom_sub = nh.subscribe("/odom", 1, &DataSaver::odomCallback, this);
    nh.getParam("target_frame", target_frame);
    nh.getParam("source_frame", source_frame);
    nh.param<double>("threshold", threshold, 2.0);
    nh.getParam("device", device); 
    distance = 0;
    odom_flag = false;
    first_flag = true;

    package_path = ros::package::getPath("depthimage_creater");
    count = 0;
}

void DataSaver::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // std::cout<<"image callback"<<std::endl;
    image = *msg;
}

void DataSaver::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // std::cout<<"cloud callback"<<std::endl;
    pc2 = *msg;
}

void DataSaver::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // std::cout<<"odom callback"<<std::endl;
    // 移動量を算出
    if(!odom_flag){
        old_odom = *msg;
        odom_flag = true;
    }
    else{
        double dt = sqrt( pow((msg->pose.pose.position.x - old_odom.pose.pose.position.x), 2) + 
                pow((msg->pose.pose.position.y - old_odom.pose.pose.position.y), 2) );
        distance += dt; 
        old_odom = *msg;
    }

    if(first_flag){
        bool flag = save_process();
        if(flag) first_flag = false;
    }

    // しきい値以上移動したらデータを保存
    if(threshold<distance){
        save_process();
        distance = 0;
    }
}

bool DataSaver::tf_listener()
{
    // tflistener
    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, now, ros::Duration(1.0));
        listener.lookupTransform(target_frame, source_frame,  now, transform);
        return true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

bool DataSaver::save_process()
{
    std::cout<<"file_path : "<<package_path<<std::endl;
    std::string file_name = std::to_string(count);

    // tf_listener
    bool success = tf_listener();
    if(!success) return false;
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double q_x = transform.getRotation().x();
    double q_y = transform.getRotation().y();
    double q_z = transform.getRotation().z();
    double q_w = transform.getRotation().w();
    // transform.setOrigin(tf::Vector3(x, y, z));
    // transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("tf:x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);

    // PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pc2, cloud);
    cloud.width = 1;
    cloud.height = cloud.points.size();

    // Image
    cv_bridge::CvImageConstPtr cv_img_ptr;
    sensor_msgs::ImageConstPtr image_ptr = boost::make_shared<sensor_msgs::Image>(image);
    try{
        cv_img_ptr = cv_bridge::toCvShare(image_ptr);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(image_ptr)->image;
    cv::Mat cv_image_rgb;
    cv::cvtColor(cv_image, cv_image_rgb, CV_BGR2RGB);

    // save PointCloud
    pcl::io::savePCDFile(package_path+"/data/cloud/"+device+"/"+file_name+".pcd", cloud);
    
    // save Image
    cv::imwrite(package_path+"/data/image/"+device+"/"+file_name+".jpg", cv_image_rgb);
    
    // save tf
    std::ofstream log;
    log.open(package_path+"/data/tf/"+device+"/"+file_name+".csv" ,std::ios::trunc);
    log << x   << ", " 
        << y   << ", "
        << z   << ", "
        << q_x << ", "
        << q_y << ", "
        << q_z << ", "
        << q_w;
    log.close();

    count++;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_saver");

    DataSaver ds;

    ros::spin();

    return 0;
}
