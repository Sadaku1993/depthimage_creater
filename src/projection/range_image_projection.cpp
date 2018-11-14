#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <omp.h>

typedef pcl::PointXYZRGB PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher image_pub;

PointCloud2 pc_msg;
bool flag = false;
void pcCallback(const PointCloud2ConstPtr msg)
{
    pc_msg = *msg;
    flag = true;
}

double distance(PointA point)
{
    double dis = sqrt( pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0));
    return dis;
}

void pcl_quicksort(CloudA &cloud, int left, int right)
{
    int l_hold, r_hold;
    double pivot;
    PointA tmp;

    l_hold = left;
    r_hold = right;
    pivot = distance(cloud.points[left]);

    tmp = cloud.points[left];

    while (left < right)
    {
        while ((distance(cloud.points[right]) >= pivot) && (left < right))
            right--;
        if (left != right)
        {
            cloud.points[left] = cloud.points[right];
            left++;
        }
        while ((distance(cloud.points[left]) <= pivot) && (left < right))
            left++;
        if (left != right)
        {
            cloud.points[right] = cloud.points[left];
            right--;
        }
    }
    cloud.points[left] = tmp;
    
    if(l_hold<left)
        pcl_quicksort(cloud, l_hold, left-1);
    if(r_hold > left)
        pcl_quicksort(cloud, left+1, r_hold);
}


void callback(const ImageConstPtr& image_msg,
              const CameraInfoConstPtr& cinfo_msg)
{
    if(!flag) return;
    
    std::cout<<"ALL GREEN"<<std::endl;
    
    // cv_bridge
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
      cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::Mat image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    image = cv_bridge::toCvShare(image_msg)->image;
    
    // camera info
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo_msg);

    // pointcloud2 to pcl
    CloudAPtr cloud(new CloudA);
    pcl::fromROSMsg(pc_msg, *cloud);

    // カメラの画角内の点群を参照点として取得
    CloudAPtr reference_cloud(new CloudA);
    for(CloudA::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
        if ((*pt).x<0) continue;
        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        if(uv.x>0 && uv.x < image.cols && uv.y > 0 && uv.y < image.rows)
            reference_cloud->points.push_back(*pt);
    }
    std::cout<<"----Image width:"<<image.cols<<" height:"<<image.rows<<std::endl;
    std::cout<<"----Cloud Size:"<<reference_cloud->points.size()<<std::endl;

    // Downsampling
    pcl::VoxelGrid<PointA> vg;
    CloudAPtr ds_cloud(new CloudA);
    vg.setInputCloud(reference_cloud);
    vg.setLeafSize (0.025f, 0.025f, 0.025f);
    vg.filter(*ds_cloud);
    std::cout<<"----Dowsample:"<<ds_cloud->points.size()<<std::endl;

    // 点群をカメラ座標系からの距離が短い順に昇順ソート
    pcl_quicksort(*ds_cloud, 0, int(ds_cloud->points.size()-1));
    
    {{{/*
    for(size_t i=0;i<ds_cloud->points.size();i++){
        for(size_t j=i+1;j<ds_cloud->points.size();j++){
            PointA p_i = ds_cloud->points[i];
            PointA p_j = ds_cloud->points[j];
            double dis_i = sqrt( pow(p_i.x, 2.0) + pow(p_i.y, 2.0) + pow(p_i.z, 2.0));
            double dis_j = sqrt( pow(p_j.x, 2.0) + pow(p_j.y, 2.0) + pow(p_j.z, 2.0));
            if(dis_i > dis_j){
                PointA tmp = ds_cloud->points[i];
                ds_cloud->points[i] = ds_cloud->points[j];
                ds_cloud->points[j] = tmp;
            }
        }
    }
    */}}}

    // 各ピクセルの距離情報を取得
    std::cout<<"----Calc distance"<<std::endl;
    int row = cv_img_ptr->image.rows;
    int col = cv_img_ptr->image.cols;

    double distance[row][col];
    bool init[row][col];
    memset(&distance, 0    , row*col);
    memset(&init,     false, row*col);

#pragma omp parallel for
    for(CloudA::iterator pt=ds_cloud->points.begin(); pt<ds_cloud->points.end(); pt++)
    {
        double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));

        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        int x = uv.x;
        int y = uv.y;
        
        if(!init[y][x]){
            distance[y][x] = range; 
            init[y][x] = true;
        }   
        else if(range<distance[y][x]){
            distance[y][x] = range;
        }
    }
    
    // Depth Colorを設定
    std::cout<<"----set depth color"<<std::endl;
    for(int y=0;y<image.rows; y++){
        for(int x=0;x<image.cols; x++){
            if(init[y][x]){
                double value = distance[y][x]/5.0;
                unsigned char r,g,b;
                pcl::visualization::FloatImageUtils::getColorForFloat (value, r, g, b);
                image.at<cv::Vec3b>(y, x)[0] = b;
                image.at<cv::Vec3b>(y, x)[1] = g;
                image.at<cv::Vec3b>(y, x)[2] = r;
            }
            else{
                image.at<cv::Vec3b>(y, x)[0] = 0;
                image.at<cv::Vec3b>(y, x)[1] = 0;
                image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "range_image_projection");
    ros::NodeHandle nh("~");
    
    image_transport::ImageTransport it(nh);
    
    ros::Subscriber pc_sub    = nh.subscribe("/cloud", 10, pcCallback); 
    
	message_filters::Subscriber<Image> image_sub(nh, "/image", 10);
	message_filters::Subscriber<CameraInfo> cinfo_sub(nh, "/camera_info", 10);
	typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cinfo_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	
    image_pub = it.advertise("/depthimage", 10);



    ros::spin();

    return 0;
}
