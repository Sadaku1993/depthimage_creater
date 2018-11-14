#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/bilateral_upsampling.h>

template<typename T_p>
class BilateralUpsampling{
    private:
        ros::NodeHandle nh;
        std::string LOAD_PATH;
        std::string SAVE_PATH;
        std::string PCD_FILE;
        std::string OUTPUT_FILE;
        
        int window_size;
        double sigma_color;
        double sigma_depth;

    public:
        BilateralUpsampling();
        void load(typename pcl::PointCloud<T_p>::Ptr& cloud);
        void save(typename pcl::PointCloud<T_p>::Ptr cloud);
        void bilateral_upsampling(typename pcl::PointCloud<T_p>::Ptr cloud,
                                  typename pcl::PointCloud<T_p>::Ptr& upsample_cloud);
        void main();
};

template<typename T_p>
BilateralUpsampling<T_p>::BilateralUpsampling()
    : nh("~")
{
    nh.getParam("load_path"     , LOAD_PATH);
    nh.getParam("save_path"     , SAVE_PATH);
    nh.getParam("pcd_file"      , PCD_FILE);
    nh.getParam("output_file"   , OUTPUT_FILE);

    nh.getParam("window_size", window_size);
    nh.getParam("sigma_color", sigma_color);
    nh.getParam("sigma_depth", sigma_depth);
}

template<typename T_p>
void BilateralUpsampling<T_p>::load(typename pcl::PointCloud<T_p>::Ptr& cloud)
{
    std::string file_name = LOAD_PATH + "/" + PCD_FILE;
    std::cout<<"-----Load :" <<file_name<<std::endl;
    
    if (pcl::io::loadPCDFile<T_p> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

template<typename T_p>
void BilateralUpsampling<T_p>::save(typename pcl::PointCloud<T_p>::Ptr cloud)
{
    typename pcl::PointCloud<T_p>::Ptr save_cloud(new pcl::PointCloud<T_p>);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    std::string file_name=SAVE_PATH+"/"+OUTPUT_FILE;

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    std::cout<<"-----Save :" <<file_name <<std::endl;
}

template<typename T_p>
void BilateralUpsampling<T_p>::bilateral_upsampling(typename pcl::PointCloud<T_p>::Ptr cloud,
                                                    typename pcl::PointCloud<T_p>::Ptr& upsample_cloud)
{
    pcl::BilateralUpsampling<T_p, T_p> bu;
    bu.setInputCloud(cloud);
    bu.setWindowSize(window_size);
    bu.setSigmaColor(sigma_color);
    bu.setSigmaDepth(sigma_depth);
    bu.setProjectionMatrix (bu.KinectSXGAProjectionMatrix);
    bu.process (*upsample_cloud);
}

template<typename T_p>
void BilateralUpsampling<T_p>::main()
{
    typename pcl::PointCloud<T_p>::Ptr cloud(new pcl::PointCloud<T_p>);
    typename pcl::PointCloud<T_p>::Ptr upsample_cloud(new pcl::PointCloud<T_p>);

    load(cloud);
    bilateral_upsampling(cloud, upsample_cloud);
    save(upsample_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bilateral_upsampling");

    BilateralUpsampling<pcl::PointXYZRGB> bu;

    bu.main();

    return 0;
}
