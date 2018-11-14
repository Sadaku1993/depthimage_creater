#include <depthimage_creater/integration.h>

ImageConstPtr image_msg;
CameraInfoConstPtr cinfo_msg;
ObjectInfoArrayConstPtr cluster_msg;
ObjectInfoArrayConstPtr object_msg;

void integration()
{
    std::cout<<"Integration"<<std::endl;
    
    // convert from ros_image to opencv_image 
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try{
    cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
    // get CamInfo
    image_geometry::PinholeCameraModel cam_model_;
    cam_model_.fromCameraInfo(cinfo_msg);

    // Debug Image
    cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

    // Integration
    ObjectInfoArray cluster_roi;     // PointCloud
    ObjectInfoArray object_roi;      // Image
    ObjectInfoArray integration_roi; // Integration
    roi_for_cluster(cluster_msg, cam_model_, cluster_roi, image);
    roi_for_bbox(object_msg, object_roi, image);
    roi_for_fusion(object_roi, cluster_roi, integration_roi, image);

    // Publish jsk_recognition_msgs
    ros::Time t = ros::Time::now();
    jsk_recognition_msgs::BoundingBoxArray object_array;
    object_array.header.frame_id = cluster_msg->header.frame_id;
    object_array.header.stamp = t;
    for(size_t i=0;i<integration_roi.object_array.size();i++){
    jsk_recognition_msgs::BoundingBox bbox;
    bbox.header.frame_id = cluster_msg->header.frame_id;
    bbox.header.stamp = t;
    bbox.pose.position = integration_roi.object_array[i].pose.position;
    bbox.pose.orientation.x = 0;
    bbox.pose.orientation.y = 0;
    bbox.pose.orientation.z = 0;
    bbox.pose.orientation.w = 1;
    bbox.dimensions.x = integration_roi.object_array[i].depth;
    bbox.dimensions.y = integration_roi.object_array[i].width;
    bbox.dimensions.z = integration_roi.object_array[i].height;
    object_array.boxes.push_back(bbox);
    }
    pub.publish(object_array);

    // Publish Image
    ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);

    // Publish PointCloud
}

double image_time, image_diff_time;
bool image_flag = false;
void callback(const ImageConstPtr im_msg,
              const CameraInfoConstPtr cf_msg)
{
    if(!image_flag){
        image_time = im_msg->header.stamp.toSec();
        image_flag = true;
    }
    else{
        image_diff_time = im_msg->header.stamp.toSec() - image_time;
        image_time = im_msg->header.stamp.toSec();
        // std::cout<<"Image:   "<<im_msg->header.stamp<<" "<<image_diff_time<<std::endl;
    }
    image_msg = im_msg;
    cinfo_msg = cf_msg;
}

double cluster_time, cluster_diff_time;
bool cluster_flag = false;
void clusterCallback(ObjectInfoArrayConstPtr msg)
{
    if(!cluster_flag){
        cluster_time = msg->header.stamp.toSec();
        cluster_flag = true;
    }
    else{
        cluster_diff_time = msg->header.stamp.toSec() - cluster_time;
        cluster_time = msg->header.stamp.toSec();
        // std::cout<<"Cluster: "<<msg->header.stamp<<" "<<cluster_diff_time<<std::endl;
    }
    cluster_msg = msg;
}

double object_time, object_diff_time;
bool object_flag = false;
void objectCallback(ObjectInfoArrayConstPtr msg)
{
    if(!object_flag){
        object_time = msg->header.stamp.toSec();
        object_flag = true;
    }
    else{
        object_diff_time = msg->header.stamp.toSec() - object_time;
        object_time = msg->header.stamp.toSec();
        // std::cout<<"Object:  "<<msg->header.stamp<<" "<<object_diff_time<<std::endl;
    }
    object_msg = msg;

    if(image_flag && cluster_flag && object_flag)
        integration();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "integration");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);

    ros::Subscriber cluster_sub = nh.subscribe("/cluster", 10, clusterCallback);
    ros::Subscriber object_sub  = nh.subscribe("/object",  10, objectCallback);
    
    message_filters::Subscriber<Image> image_sub(nh, "/image", 10);
    message_filters::Subscriber<CameraInfo> cinfo_sub(nh, "/camera_info", 10);
    typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, cinfo_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bbox_output", 10);
    image_pub = it.advertise("/image_output", 10);
    
    ros::spin();

    return 0;
}
