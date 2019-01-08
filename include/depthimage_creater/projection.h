#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <fstream>
using namespace std;

template<typename T_p, typename T_c, typename T_ptr>
class Projection{
    private:

    public:
        void projection(T_ptr cloud, 
                cv::Mat image, 
                sensor_msgs::CameraInfo cinfo,
                T_ptr &output_cloud,
                cv::Mat &coutput_image,
                std::string distance_name);
};

template<typename T_p, typename T_c, typename T_ptr>
void Projection<T_p, T_c, T_ptr>::projection(T_ptr cloud,
        cv::Mat image,
        sensor_msgs::CameraInfo cinfo,
        T_ptr &output_cloud,
        cv::Mat &output_image)
{
    // camera info
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cinfo);

    // カメラの画角内の点群を参照点として取得
    T_ptr reference_cloud(new T_c);
    for(typename T_c::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
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

    // 各ピクセルの距離情報を取得
    std::cout<<"----Calc distance"<<std::endl;
    int row = image.rows;
    int col = image.cols;

    double distance[row][col];
    double info[row][col][3]; // x, y, z, distance
    bool init[row][col];
    memset(&distance, 0    , row*col);
    memset(&info,     0    , row*col*3);
    memset(&init,     false, row*col);

#pragma omp parallel for
    for(typename T_c::iterator pt=reference_cloud->points.begin(); pt<reference_cloud->points.end(); pt++)
    {
        double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));

        cv::Point3d pt_cv(-(*pt).y, -(*pt).z, (*pt).x);
        cv::Point2d uv;
        uv = cam_model.project3dToPixel(pt_cv);

        int x = uv.x;
        int y = uv.y;

        if((*pt).z<-1.0){
            for(int i=-2;i<=2;i++){
                for(int j=-2;j<=2;j++){
                    if(0<=x+i && x+i<col && 0<=y+j && y+j<row){
                        if(!init[y+j][x+i]){
                            distance[y+j][x+i] = range;
                            info[y+j][x+i][0] = -(*pt).y;
                            info[y+j][x+i][1] = -(*pt).z;
                            info[y+j][x+i][2] = (*pt).x;
                            info[y+j][x+i][3] = range;

                            init[y+j][x+i] = true;
                        }
                        else if(range<distance[y+j][x+i]){
                            distance[y+j][x+i] = range;
                            info[y+j][x+i][0] = -(*pt).y;
                            info[y+j][x+i][1] = -(*pt).z;
                            info[y+j][x+i][2] = (*pt).x;
                            info[y+j][x+i][3] = range;
                        }
                    }
                }
            }
        }else{
            for(int i=-1;i<=1;i++){
                for(int j=-1;j<=1;j++){
                    if(0<=x+i && x+i<col && 0<=y+j && y+j<row){
                        if(!init[y+j][x+i]){
                            distance[y+j][x+i] = range;
                            info[y+j][x+i][0] = -(*pt).y;
                            info[y+j][x+i][1] = -(*pt).z;
                            info[y+j][x+i][2] = (*pt).x;
                            info[y+j][x+i][3] = range;

                            init[y+j][x+i] = true;
                        }
                        else if(range<distance[y+j][x+i]){
                            distance[y+j][x+i] = range;
                            info[y+j][x+i][0] = -(*pt).y;
                            info[y+j][x+i][1] = -(*pt).z;
                            info[y+j][x+i][2] = (*pt).x;
                            info[y+j][x+i][3] = range;

                        }
                    }
                }
            }
        }



        /*
        if(!init[y][x]){
            distance[y][x] = range; 
            init[y][x] = true;
        }
        else if(range<distance[y][x]){
            distance[y][x] = range;
        }
        */
    }

    // Depth Colorを設定
    std::cout<<"----set depth color"<<std::endl;
    for(int y=0;y<image.rows; y++){
        for(int x=0;x<image.cols; x++){
            if(init[y][x]){
                double value = distance[y][x]/50.0;
                unsigned char r,g,b;
                pcl::visualization::FloatImageUtils::getColorForFloat (value, r, g, b);
                output_image.at<cv::Vec3b>(y, x)[0] = b;
                output_image.at<cv::Vec3b>(y, x)[1] = g;
                output_image.at<cv::Vec3b>(y, x)[2] = r;
            }
            else{
                output_image.at<cv::Vec3b>(y, x)[0] = 0;
                output_image.at<cv::Vec3b>(y, x)[1] = 0;
                output_image.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    // Distanceを保存
    std::ofstream ofs(distance_name, std::ios::trunc);
    for(int y=0;y<image.rows; y++){
      for(int x=0;x<image.cols; x++){
        if(init[y][x]) ofs << distance[y][x] << ",";
        else ofs << 0.0 << ",";
      }
      ofs << std::endl;
    }
    ofs.close();

    *output_cloud += *reference_cloud;
}
