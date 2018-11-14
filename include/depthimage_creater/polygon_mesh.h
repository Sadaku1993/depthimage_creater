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
        double normal_search_radius;  //default 0.6
        double polygon_serach_radius; //default 1.0

    public:
        PolygonMesh();

        void set_param(double n_radius, double p_radius);

        void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_normal);

        void polygon_mesh(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                          pcl::PolygonMesh& triangles);
        void main(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);
};

PolygonMesh::PolygonMesh()
{
    normal_search_radius = 0.6;
    polygon_serach_radius = 1.0;
}

void PolygonMesh::set_param(double n_radius, double p_radius)
{
    normal_search_radius = n_radius;
    polygon_serach_radius = p_radius;
}

void PolygonMesh::normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_normal)
{
	std::cout<<"------Normal Estimation"<<std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (normal_search_radius);
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
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree2->setInputCloud (cloud);
    
    pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
    gp3.setSearchRadius (polygon_serach_radius);

    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
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

void PolygonMesh::main(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
{
	std::cout<<"------PolygonMesh"<<std::endl;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    normal_estimation(cloud, normal_cloud);

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

        // 三角形の内側に点がある場合:1, ない場合:0
        // int check = hittest_point_polygon_3d(x0, x1, x2, x0);
        // std::cout<<check<<std::endl;

        Eigen::Vector3f centroid;
        centroid[0] = (p0.x+p1.x+p2.x)/3.0;
        centroid[1] = (p0.y+p1.y+p2.y)/3.0;
        centroid[2] = (p0.z+p1.z+p2.z)/3.0;
        // std::cout<<"centroid:"<<std::endl;
        // std::cout<<centroid<<std::endl;

        // PolygonのMin-Maxを算出
        // std::vector<double> vec_x(3), vec_y(3), vec_z(3);
        // vec_x[0] = p0.x; vec_x[1] = p1.x; vec_x[2] = p2.x;
        // vec_y[0] = p0.y; vec_y[1] = p1.y; vec_y[2] = p2.y;
        // vec_z[0] = p0.z; vec_z[1] = p1.z; vec_z[2] = p2.z;
        // double min_x = *std::min_element(vec_x.begin(), vec_x.end());
        // double min_y = *std::min_element(vec_y.begin(), vec_y.end());
        // double min_z = *std::min_element(vec_z.begin(), vec_z.end());
        // double max_x = *std::max_element(vec_x.begin(), vec_x.end());
        // double max_y = *std::max_element(vec_y.begin(), vec_y.end());
        // double max_z = *std::max_element(vec_z.begin(), vec_z.end());
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

    pcl::copyPointCloud(*normal_cloud, *output_cloud);
}
