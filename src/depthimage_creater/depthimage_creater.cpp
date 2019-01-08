/*
 *
 * depthimage_creater.cpp
 *
 * PointCloud, Image, TF, camera_info, mapデータからdepthimageを作成
 *
 * author:Yudai Sadakuni
 *
 */

#include <ros/ros.h>
#include <ros/package.h> 
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <depthimage_creater/quicksort.h>
#include <depthimage_creater/projection.h>

std::vector<std::string> split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

template<typename T_p, typename T_c, typename T_ptr>
class DepthImageCreater{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub_tf;
        ros::Publisher pub_cloud;

        std::string package_path;
        std::string device;
        std::string map_path;
        std::string map_frame;
        std::string camera_frame;
        std::string lidar_frame;
        int count;

        sensor_msgs::CameraInfo cinfo;
        int cinfo_height;
        int cinfo_width;
        std::string cinfo_distortion_model;
        std::string cinfo_frame_id;
        std::vector<double> D;
        std::vector<double> K;
        std::vector<double> R;
        std::vector<double> P;
        int binning_x;
        int binning_y;
        int roi_x_offset;
        int roi_y_offset;
        int roi_height;
        int roi_width;
        bool roi_do_rectify;

        // Upsample param
        double normal_serach_radius;
        double polygon_serach_radius;

    public:
        DepthImageCreater();
        int file_count_boost(const boost::filesystem::path& root);
        bool loadImage(cv::Mat& image, std::string file_name);
        bool loadCloud(T_ptr& cloud, std::string file_name);
        bool loadTF(tf::Transform& transform, std::string file_name);
        void main();
        void depthimage_creater(cv::Mat image, 
                                T_ptr map, 
                                T_ptr cloud, 
                                std::string depthimage_name
                                std::string distance_name);
        void show_transform(std::string parent_frame,
                            std::string child_frame, 
                            tf::Transform transform);
        void transform_pointcloud(T_ptr cloud, 
                                  T_ptr& trans_cloud, 
                                  tf::Transform transform, 
                                  std::string target_frame);
        bool tflistener(std::string target_frame, 
                        std::string source_frame, 
                        tf::TransformListener& listener, 
                        tf::StampedTransform& transform);
        void tfbroadcast(std::string target_frame, 
                         std::string source_frame, 
                         tf::Transform transform);
        void camerainfo();

        QuickSort<T_p, T_c, T_ptr> qs;
        Projection<T_p, T_c, T_ptr> pj;
};

template<typename T_p, typename T_c, typename T_ptr>
DepthImageCreater<T_p, T_c, T_ptr>::DepthImageCreater()
    : nh("~")
{
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    pub_tf = nh.advertise<geometry_msgs::Transform>("/transform", 1);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud/test", 1);

    package_path = ros::package::getPath("depthimage_creater");
    
    nh.getParam("device", device);
    nh.getParam("map_path", map_path);
    nh.getParam("map_frame", map_frame);
    nh.getParam("camera_frame", camera_frame);
    nh.getParam("lidar_frame", lidar_frame);

    // camera info
    nh.getParam("height", cinfo_height);
    nh.getParam("width", cinfo_width);
    nh.getParam("distortion_model", cinfo_distortion_model);
    nh.getParam("header/frame_id", cinfo_frame_id);
    nh.getParam("D", D);
    nh.getParam("K", K);
    nh.getParam("R", R);
    nh.getParam("P", P);
    nh.getParam("binning_x", binning_x);
    nh.getParam("binning_y", binning_y);
    nh.getParam("roi/x_offset", roi_x_offset);
    nh.getParam("roi/y_offset", roi_y_offset);
    nh.getParam("roi/height", roi_height);
    nh.getParam("roi/width", roi_width);
    nh.getParam("roi/do_rectify", roi_do_rectify);

    //Upsample param
    nh.getParam("normal_serach_radius", normal_serach_radius);
    nh.getParam("polygon_serach_radius", polygon_serach_radius);

    count = 0;
}

// path内にデータがいくつあるか確認
template<typename T_p, typename T_c, typename T_ptr>
int DepthImageCreater<T_p, T_c, T_ptr>::file_count_boost(const boost::filesystem::path& root) {
    namespace fs = boost::filesystem;
    if (!fs::exists(root) || !fs::is_directory(root)) return 0;
    int result = 0;
    fs::directory_iterator last;
    for (fs::directory_iterator pos(root); pos != last; ++pos) {
        ++result;
        if (fs::is_directory(*pos)) result += file_count_boost(pos->path());
    }
    return result;
}

// Imageデータを読み込み
template<typename T_p, typename T_c, typename T_ptr>
bool DepthImageCreater<T_p, T_c, T_ptr>::loadImage(cv::Mat& image, std::string file_name)
{
    // std::cout<<"Load Image"<<std::endl;
    image = cv::imread(file_name, 1);
    
    if(image.empty()){
        std::cout<<file_name<<" is none"<<std::endl;
        return false;
    }
    else{
        std::cout<<"Load "<<file_name<<" is success"<<std::endl;
        return true;
    }
}

// PCDデータを読み込み
template<typename T_p, typename T_c, typename T_ptr>
bool DepthImageCreater<T_p, T_c, T_ptr>::loadCloud(T_ptr& cloud, std::string file_name)
{
    // std::cout<<"Load Cloud"<<std::endl;
    if (pcl::io::loadPCDFile<T_p> (file_name, *cloud) == -1) //* load the file
    {
        std::cout<<file_name<<" is none"<<std::endl;
        return false;
    }
    else{
        std::cout<<"Load "<<file_name<<" is success"<<std::endl;
        return true;
    }
}

// TFデータを読み込み
template<typename T_p, typename T_c, typename T_ptr>
bool DepthImageCreater<T_p, T_c, T_ptr>::loadTF(tf::Transform& transform, std::string file_name)
{
    // std::cout<<"Load TF"<<std::endl;
    double x; double y; double z;
    double q_x; double q_y; double q_z; double q_w;
    bool flag = false;

    std::ifstream ifs(file_name);
    std::string line;
    while(getline(ifs, line)){
            std::vector<std::string> strvec = split(line , ',');
            x = stof(strvec.at(0));
            y = stof(strvec.at(1));
            z = stof(strvec.at(2));
            q_x = stof(strvec.at(3));
            q_y = stof(strvec.at(4));
            q_z = stof(strvec.at(5));
            q_w = stof(strvec.at(6));
            flag = true;
    }

    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(q_x, q_y, q_z, q_w));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q_x, q_y, q_z, q_w)).getRPY(roll ,pitch, yaw);
    printf("tf:x:%.2f y:%.2f z:%.2f roll:%.2f pitch:%.2f yaw:%.2f\n", x, y, z, roll, pitch, yaw);
    return flag;
}


// main
template<typename T_p, typename T_c, typename T_ptr>
void DepthImageCreater<T_p, T_c, T_ptr>::main()
{
    std::string cloud_path = package_path+"/data/cloud/"+device;
    std::string image_path = package_path+"/data/image/"+device;
    std::string tf_path = package_path+"/data/tf/"+device;
    std::string depthimage_path = package_path+"/data/depthimage/"+device;
    std::string distance_path = package_path+"/data/distance/"+device;

    int file_size = file_count_boost(cloud_path.c_str());
    std::cout<<"file_size:"<<file_size<<std::endl;

    // CameraInfo読み込み
    camerainfo();
    std::cout<<"CameraInfo:"<<cinfo<<std::endl;

    // 地図の読み込み
    T_ptr map(new T_c);
    if(!loadCloud(map, map_path))
         return;


    for(int i=0;i<file_size;i++){
        T_ptr cloud(new T_c);
        cv::Mat image;
        tf::Transform transform;

        std::string cloud_name=cloud_path+"/"+std::to_string(count)+".pcd";
        std::string image_name=image_path+"/"+std::to_string(count)+".jpg";
        std::string tf_name=tf_path+"/"+std::to_string(count)+".csv";
        std::string depthimage_name=depthimage_path+"/"+std::to_string(count)+".jpg";
        std::string distance_name=distance_path+"/"+std::to_strin(count)+".csv";

        bool cloud_flag = loadCloud(cloud, cloud_name);
        bool image_flag = loadImage(image, image_name);
        bool tf_flag = loadTF(transform, tf_name);

        if(!cloud_flag || !image_flag || !tf_flag){
            std::cout<<"Node:"<<i<<" fail..."<<std::endl;
        }
        else{
            std::cout<<"start depthimage create"<<std::endl;
            
            // transform publisher
            geometry_msgs::Transform transform_msg;
            tf::transformTFToMsg(transform, transform_msg);
            pub_tf.publish(transform_msg);
            
            // depthimage creater
            depthimage_creater(image, map, cloud, depthimage_name, distance_name);
        }
        
        // publish pose
        T_ptr pose(new T_c);
        T_p point;
        point.x = transform.getOrigin().x();
        point.y = transform.getOrigin().y();
        point.z = transform.getOrigin().z();
        pose->points.push_back(point);
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*pose, pc2);
        pc2.header.frame_id = map_frame;
        pc2.header.stamp = ros::Time::now();
        pub.publish(pc2);

        count++;
    }
}

template<typename T_p, typename T_c, typename T_ptr>
void DepthImageCreater<T_p, T_c, T_ptr>::depthimage_creater(cv::Mat image,
                                                            T_ptr map,
                                                            T_ptr cloud,
                                                            std::string depthimage_name,
                                                            std::string distance_name)
{
    std::cout<<"make depthimage ..."<<std::endl;
    
    //tf listener ( camera_frame --> map_frame)
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    tflistener(camera_frame, map_frame, listener, stamped_transform);
    tf::Transform transform;
    transform.setOrigin(stamped_transform.getOrigin());
    transform.setRotation(stamped_transform.getRotation());
    show_transform(camera_frame, map_frame, transform);

    // transform map pointcloud (camera_frame)
    T_ptr map_transcloud(new T_c);
    transform_pointcloud(map, map_transcloud, transform, camera_frame);

    
    // Quicksort
    clock_t start, end;
    start = clock();
    T_ptr sort_cloud(new T_c);
    qs.split_sort(*map_transcloud, *sort_cloud);
    end = clock();
    printf("split quicksort %5.2f秒かかりました (Size:%d)\n", 
          (double)(end-start)/CLOCKS_PER_SEC, int(sort_cloud->points.size()));

    // projection
    std::cout<<"Image cols:"<<image.cols<<" rows:"<<image.rows<<std::endl;
    cv::Mat output_image = image.clone();
    T_ptr reference_cloud(new T_c);

    pj.projection(sort_cloud, image, cinfo, reference_cloud, output_image);

    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*reference_cloud, pc2);
    pc2.header.frame_id = camera_frame;
    pc2.header.stamp = ros::Time::now();
    pub_cloud.publish(pc2);

    // cv::imshow("DepthImage", output_image);
    // cv::waitKey(10);

    cv::imwrite(depthimage_name, output_image);
}


// show transform
template<typename T_p, typename T_c, typename T_ptr>
void DepthImageCreater<T_p, T_c, T_ptr>::show_transform(std::string parent_frame,
                                                        std::string child_frame,
                                                        tf::Transform transform)
{
    tf::Vector3 v = transform.getOrigin();
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    tf::Quaternion q = transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll ,pitch, yaw);
    std::cout<<parent_frame<<"-->"<<child_frame<<" "
             <<"x:"<<x
             <<"y:"<<y
             <<"z:"<<z
             <<"roll:"<<roll
             <<"pitch:"<<pitch
             <<"yaw:"<<yaw<<std::endl;
}

// transform pointcloud
template<typename T_p, typename T_c, typename T_ptr>
void DepthImageCreater<T_p, T_c, T_ptr>::transform_pointcloud(T_ptr cloud,
                                                              T_ptr& trans_cloud,
                                                              tf::Transform transform,
                                                              std::string target_frame)
{
    pcl_ros::transformPointCloud(*cloud, *trans_cloud, transform);
    trans_cloud->header.frame_id = target_frame;
}

// transform listener
template<typename T_p, typename T_c, typename T_ptr>
bool DepthImageCreater<T_p, T_c, T_ptr>::tflistener(std::string target_frame,
                                                    std::string source_frame,
                                                    tf::TransformListener& listener,
                                                    tf::StampedTransform& transform)
{
    std::cout<<"tflistener"<<std::endl;
    try{
        // ros::Time time = ros::Time::now();
        ros::Time time = ros::Time(0);
        listener.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
        listener.lookupTransform(target_frame, source_frame, time, transform);
        return true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

// transform broadcast
template<typename T_p, typename T_c, typename T_ptr>
void DepthImageCreater<T_p, T_c, T_ptr>::tfbroadcast(std::string parent_frame,
                                                     std::string child_frame,
                                                     tf::Transform transform)
{
    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}

// camerainfo
template<typename T_p, typename T_c, typename T_ptr>
void DepthImageCreater<T_p, T_c, T_ptr>::camerainfo()
{
    cinfo.header.stamp = ros::Time::now();
    cinfo.header.frame_id = cinfo_frame_id;
    cinfo.height = cinfo_height;
    cinfo.width = cinfo_width;
    cinfo.distortion_model = cinfo_distortion_model;
    cinfo.D = D;
    for(size_t i=0;i<cinfo.K.size();i++)
        cinfo.K[i] = K[i];
    for(size_t i=0;i<cinfo.R.size();i++)
        cinfo.R[i] = R[i];
    for(size_t i=0;i<cinfo.P.size();i++)
        cinfo.P[i] = P[i];
    cinfo.binning_x = binning_x;
    cinfo.binning_y = binning_y;
    cinfo.roi.x_offset = roi_x_offset;
    cinfo.roi.y_offset = roi_y_offset;
    cinfo.roi.height = roi_height;
    cinfo.roi.width = roi_width;
    cinfo.roi.do_rectify = roi_do_rectify;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage_creater");

    DepthImageCreater<pcl::PointXYZINormal, pcl::PointCloud<pcl::PointXYZINormal>, pcl::PointCloud<pcl::PointXYZINormal>::Ptr> dic;

    dic.main();

    return 0;
}
