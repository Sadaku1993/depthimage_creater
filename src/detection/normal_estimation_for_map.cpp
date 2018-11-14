#include <ros/ros.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

class NormalEstimation{
    private:
        ros::NodeHandle nh;

        string LOAD_PATH;
        string SAVE_PATH;
        string PCD_FILE;
        string OUTPUT_FILE;
        double search_radius;

    public:
        NormalEstimation();

        void load(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void save(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud);

        void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_normal);
        void main();
};

NormalEstimation::NormalEstimation()
    : nh("~")
{
    nh.getParam("search_radius", search_radius);
    nh.getParam("load_path"    , LOAD_PATH);
    nh.getParam("save_path" , SAVE_PATH);
    nh.getParam("pcd_file", PCD_FILE);
    nh.getParam("output_file", OUTPUT_FILE);
}

void NormalEstimation::load(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    string file_name = LOAD_PATH + "/" + PCD_FILE;
    cout<<"-----Load :" <<file_name<<endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void NormalEstimation::save(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    string file_name=SAVE_PATH+"/"+OUTPUT_FILE;

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    cout<<"-----Save :" <<file_name <<endl;
}

void NormalEstimation::normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_normal)
{
	cout<<"-----Normal Estimation"<<endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (search_radius);
    ne.compute(*normals);

    pcl::PointXYZINormal tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		// tmp.r = cloud->points[i].r;
		// tmp.g = cloud->points[i].g;
		// tmp.b = cloud->points[i].b;
		if(!isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		cloud_normal->points.push_back(tmp);
	}
}

void NormalEstimation::main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    load(load_cloud);
    normal_estimation(load_cloud, normal_cloud);
    save(normal_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_local");
    
    NormalEstimation ne;

    ne.main();

    return 0;
}
