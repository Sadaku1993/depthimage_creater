#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

template<typename T_p, typename T_c, typename T_ptr>
class OutlierRemoval{
    private:
        ros::NodeHandle nh;

        std::string LOAD_PATH;
        std::string SAVE_PATH;
        std::string PCD_FILE;
        std::string OUTPUT_FILE;

        double mean_K;
        double std_dev_mul;

    public:
        OutlierRemoval();

        void load(T_ptr& cloud);
        void save(T_ptr cloud);
        void remove_outliers(T_ptr cloud, T_ptr& cloud_filtered);

        void main();
};

template<typename T_p, typename T_c, typename T_ptr>
OutlierRemoval<T_p, T_c, T_ptr>::OutlierRemoval()
    : nh("~")
{
    nh.getParam("load_path"    , LOAD_PATH);
    nh.getParam("save_path"    , SAVE_PATH);
    nh.getParam("pcd_file"     , PCD_FILE);
    nh.getParam("output_file"  , OUTPUT_FILE);
    nh.getParam("mean_K", mean_K);
    nh.getParam("std_dev_mul", std_dev_mul);
}

template<typename T_p, typename T_c, typename T_ptr>
void OutlierRemoval<T_p, T_c, T_ptr>::load(T_ptr& cloud)
{
    std::string file_name = LOAD_PATH + "/" + PCD_FILE;
    std::cout<<"-----Load :" <<file_name<<std::endl;
    
    if (pcl::io::loadPCDFile<T_p> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

template<typename T_p, typename T_c, typename T_ptr>
void OutlierRemoval<T_p, T_c, T_ptr>::save(T_ptr cloud)
{
    T_ptr save_cloud(new T_c);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    std::string file_name=SAVE_PATH+"/"+OUTPUT_FILE;

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    std::cout<<"-----Save :" <<file_name <<std::endl;
}

template<typename T_p, typename T_c, typename T_ptr>
void OutlierRemoval<T_p, T_c, T_ptr>::remove_outliers(T_ptr cloud, T_ptr& cloud_filtered)
{
    std::cout<<"-----OutlierRemoval"<<std::endl;
    pcl::StatisticalOutlierRemoval<T_p> sor;
    sor.setInputCloud (cloud);//外れ値を除去する点群を入力
    sor.setMeanK (mean_K);//MeanKを設定
    sor.setStddevMulThresh (std_dev_mul);
    sor.setNegative (false);//外れ値を出力する場合はtrueにする
    sor.filter (*cloud_filtered);//出力
}

template<typename T_p, typename T_c, typename T_ptr>
void OutlierRemoval<T_p, T_c, T_ptr>::main()
{
    T_ptr cloud(new T_c);
    T_ptr cloud_filtered(new T_c);

    load(cloud);
    remove_outliers(cloud, cloud_filtered);
    save(cloud_filtered);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "outlier_removal");

    OutlierRemoval<PointA, CloudA, CloudAPtr>OR;

    OR.main();

    return 0;
}
