#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <amsl_recog_msgs/ObjectInfoWithROI.h>
#include <amsl_recog_msgs/ObjectInfoArray.h>

using namespace std;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace amsl_recog_msgs;

ros::Publisher pub;

bool image_flag = false;

PointCloud2ConstPtr pc_msg_;
void pc_callback(const PointCloud2ConstPtr msg)
{
    pc_msg_ = msg;
}

ImageConstPtr image_msg_;
CameraInfoConstPtr cinfo_msg_;
void callback(const ImageConstPtr &image_msg,
              const CameraInfoConstPtr &cinfo_msg)
{
    image_msg_ = image_msg;
    cinfo_msg_ = cinfo_msg;
    image_flag = true;
}

void object_callback(const ObjectInfoArrayConstPtr object_msg)
{
    int object_size = int(object_msg->object_array.size());
    cout<<"object_size:"<<object_size<<endl;

    if (!image_flag) return;

    // cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
        cv_img_ptr = cv_bridge::toCvShare(image_msg_);
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg_)->image;

    // camera info
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr area(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg_, *cloud);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud->points.begin(); pt < cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);
        
        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows)
        {
            bool flag = false;
            for(int i=0;i<object_msg->object_array.size();i++)
            {
                ObjectInfoWithROI roi = object_msg->object_array[i];
                if(roi.Class=="person" || roi.Class=="car" || roi.Class=="bus" || roi.Class=="bicycle")
                {
                    if(roi.xmin-20<uv.x && uv.x<roi.xmax+20 && roi.ymin-20<uv.y && uv.y<roi.ymax+20)
                        flag = true;
                }
            }
            if(!flag)
                area->points.push_back(*pt);
        }
    }

    PointCloud2 rm_object;
    pcl::toROSMsg(*area, rm_object);
    rm_object.header.frame_id = object_msg->header.frame_id;
    rm_object.header.stamp = ros::Time::now();
    pub.publish(rm_object);
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "rm_object");

    ros::NodeHandle nh("~");

    ros::Subscriber cloud_sub  = nh.subscribe("/cloud", 10, pc_callback);
    ros::Subscriber object_sub = nh.subscribe("/objectinfo", 10, object_callback); 

    message_filters::Subscriber<Image> image_sub(nh, "/image", 10);
    message_filters::Subscriber<CameraInfo> cinfo_sub(nh, "/camera_info", 10);
    
    typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cinfo_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

    pub = nh.advertise<PointCloud2>("/rm_object", 10);

    ros::spin();

    return 0;
}

