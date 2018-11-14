#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <iostream>
#include <depthimage_creater/plane_equataion.h>
#include <depthimage_creater/check_area.h>

class PolygonMesh{
    private:
        ros::NodeHandle nh;

        std::string LOAD_PATH;
        std::string SAVE_PATH;
        std::string PCD_FILE;
        std::string OUTPUT_FILE;
        std::string VTK_FILE;
        double search_radius;

        double gp3_serach_radius;
        double gp3_K;

    public:
        PolygonMesh();

        void load(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
        void save(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                  pcl::PolygonMesh polygonmesh);

        void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_normal);

        void polygon_mesh(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                          pcl::PolygonMesh& triangles);
        void main();
};

PolygonMesh::PolygonMesh()
    : nh("~")
{
    nh.getParam("search_radius", search_radius);
    nh.getParam("load_path"    , LOAD_PATH);
    nh.getParam("save_path"    , SAVE_PATH);
    nh.getParam("pcd_file"     , PCD_FILE);
    nh.getParam("vtk_file"     , VTK_FILE);
    nh.getParam("output_file"  , OUTPUT_FILE);
    nh.getParam("gp3_serach_radius", gp3_serach_radius);
    nh.getParam("gp3_K", gp3_K);
}

void PolygonMesh::load(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::string file_name = LOAD_PATH + "/" + PCD_FILE;
    std::cout<<"-----Load :" <<file_name<<std::endl;
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("-----Couldn't read file\n");
    }
}

void PolygonMesh::save(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                       pcl::PolygonMesh polygonmesh)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::copyPointCloud(*cloud, *save_cloud);

    save_cloud->width = 1;
    save_cloud->height = save_cloud->points.size();

    std::string file_name=SAVE_PATH+"/"+OUTPUT_FILE;
    std::string vtk_name=SAVE_PATH+"/"+VTK_FILE;

    pcl::io::savePCDFileASCII(file_name, *save_cloud);
    pcl::io::saveVTKFile(vtk_name, polygonmesh);
    std::cout<<"-----Save :" <<file_name <<std::endl;
}

void PolygonMesh::normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_normal)
{
	std::cout<<"-----Normal Estimation"<<std::endl;
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
		if(!std::isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!std::isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		cloud_normal->points.push_back(tmp);
	}
}

void PolygonMesh::polygon_mesh(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                               pcl::PolygonMesh& triangles)
{
	std::cout<<"-----Polygon Mesh"<<std::endl;

    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree2->setInputCloud (cloud);
    
    pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
    gp3.setSearchRadius (gp3_serach_radius);

    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (gp3_K);
    gp3.setMaximumSurfaceAngle(M_PI/4);
    gp3.setMinimumAngle(M_PI/18);
    gp3.setMaximumAngle(2*M_PI/3);
    gp3.setNormalConsistency(false);

    gp3.setInputCloud (cloud);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
}

void PolygonMesh::main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    load(load_cloud);
    normal_estimation(load_cloud, normal_cloud);

    pcl::PolygonMesh triangles;
    polygon_mesh(normal_cloud, triangles);
    for(size_t i=0;i<triangles.polygons.size();i++){
        // std::cout<<"polygon:"<<i<<std::endl;
        // std::cout<<"  0-->"<<triangles.polygons[i].vertices[0]<<" "<<normal_cloud->points[triangles.polygons[i].vertices[0]]<<std::endl;
        // std::cout<<"  1-->"<<triangles.polygons[i].vertices[1]<<" "<<normal_cloud->points[triangles.polygons[i].vertices[1]]<<std::endl;
        // std::cout<<"  2-->"<<triangles.polygons[i].vertices[2]<<" "<<normal_cloud->points[triangles.polygons[i].vertices[2]]<<std::endl;

        pcl::PointXYZINormal p0, p1, p2;
        p0 = normal_cloud->points[triangles.polygons[i].vertices[0]];
        p1 = normal_cloud->points[triangles.polygons[i].vertices[1]];
        p2 = normal_cloud->points[triangles.polygons[i].vertices[2]];

        Vector3D v0(p0.x, p0.y, p0.z);
        Vector3D v1(p1.x, p1.y, p1.z);
        Vector3D v2(p2.x, p2.y, p2.z);

        Plane plane = CreatePlaneFromPolygon(v0, v1, v2);

        // std::cout<<"a:"<<plane.a<<" "
        //          <<"b:"<<plane.b<<" "
        //          <<"c:"<<plane.c<<" "
        //          <<"d:"<<plane.d<<std::endl;

        double result;
        result = plane.a * p0.x + plane.b * p0.y + plane.c * p0.z;
        // std::cout<<"result:"<<result<<std::endl;
        // std::cout<<"d:"<<plane.d<<std::endl;

        VectorXD x0, x1, x2;
        x0.x = p0.x; x0.y = p0.y; x0.z = p0.z;
        x1.x = p1.x; x1.y = p1.y; x1.z = p1.z;
        x2.x = p2.x; x2.y = p2.y; x2.z = p2.z;

        int check = hittest_point_polygon_3d(x0, x1, x2, x0);
        // 三角形の内側に点がある場合:1, ない場合:0
        // std::cout<<check<<std::endl;

        Eigen::Vector3f centroid;
        centroid[0] = (p0.x+p1.x+p2.x)/3.0;
        centroid[1] = (p0.y+p1.y+p2.y)/3.0;
        centroid[2] = (p0.z+p1.z+p2.z)/3.0;
        // std::cout<<"centroid:"<<std::endl;
        // std::cout<<centroid<<std::endl;

        std::vector<double> vec_x(3), vec_y(3), vec_z(3);
        vec_x[0] = p0.x; vec_x[1] = p1.x; vec_x[2] = p2.x;
        vec_y[0] = p0.y; vec_y[1] = p1.y; vec_y[2] = p2.y;
        vec_z[0] = p0.z; vec_z[1] = p1.z; vec_z[2] = p2.z;
        double min_x = *std::min_element(vec_x.begin(), vec_x.end());
        double min_y = *std::min_element(vec_y.begin(), vec_y.end());
        double min_z = *std::min_element(vec_z.begin(), vec_z.end());
        double max_x = *std::max_element(vec_x.begin(), vec_x.end());
        double max_y = *std::max_element(vec_y.begin(), vec_y.end());
        double max_z = *std::max_element(vec_z.begin(), vec_z.end());

        // std::cout<<"min_x:"<<min_x<<" "
        //          <<"min_y:"<<min_y<<" "
        //          <<"min_z:"<<min_z<<" "
        //          <<"max_x:"<<max_x<<" "
        //          <<"max_y:"<<max_y<<" "
        //          <<"max_z:"<<max_z<<std::endl;
        pcl::PointXYZINormal point;
        point.x = centroid[0]; point.y = centroid[1]; point.z = centroid[2];
        normal_cloud->points.push_back(point);
    }

    save(normal_cloud, triangles);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polygon_mesh");
    
    PolygonMesh ne;

    ne.main();

    return 0;
}
