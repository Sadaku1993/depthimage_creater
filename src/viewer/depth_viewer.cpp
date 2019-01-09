#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

using namespace std;

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

class Viewer{
    private:
        ros::NodeHandle nh;
        
        std::string package_path;
        std::string image_path;
        std::string distance_path;
        std::string device;

        int image_row;
        int image_col;

        vector< vector<double> > distance;

    public:
        Viewer();
        bool loadImage(cv::Mat& image, std::string file_name);
        bool loadDistance(std::string file_name);
        void main();
};


Viewer::Viewer()
    : nh("~")
{
    package_path = ros::package::getPath("depthimage_creater");
    nh.param<std::string>("device", device, "realsense0");
    nh.param<int>("image_row", image_row, 720);
    nh.param<int>("image_col", image_col, 1280);

    image_path = package_path + "/data/image/" + device;
    distance_path = package_path + "/data/distance/" + device;
};

bool Viewer::loadImage(cv::Mat& image, std::string file_name)
{
    image = cv::imread(file_name, 1);
    
    if(image.empty()){
        std::cout<<"Image is none"<<std::endl;
        return false;
    }
    else{
        std::cout<<"Load Image Success"<<std::endl;
        return true;
    }
}


bool Viewer::loadDistance(std::string file_name)
{
    std::ifstream ifs(file_name);

    if(ifs.fail()){
        std::cout<< "File is none" <<std::endl;
        return false;
    }

    std::string line;
    int row = 0;
    while(getline(ifs, line)){
        std::vector<std::string> strvec = split(line, ',');
        for(size_t i=0;i<strvec.size();i++){
            distance[row][i] = std::stod(strvec.at(i));
        }
        row++;
    }
}

void Viewer::main()
{
	int num = 0;
    cout << "Num -> ";
    cin >> num;

    std::string image_name = image_path + "/" + std::to_string(num) + ".jpg";
    std::string distance_name = distance_path + "/" + std::to_string(num) + ".csv";

    std::cout<<"image : "<<image_name<<std::endl;
    std::cout<<"distance : "<<distance_name<<std::endl;

    cv::Mat image;
    if(!loadImage(image, image_name))
        exit(0);

    image_row = image.rows;
    image_col = image.cols;

    distance.assign(image_row, std::vector<double>(image_col, 0));
    std::cout << distance.size() << std::endl;
    std::cout << distance.front().size() << std::endl;

        
    if(!loadDistance(distance_name))
        exit(1);

    while(true)
    {
        int row = 0;
        int col = 0;
        std::cout<<" row : ";
        cin >> row;

        std::cout<<" col : ";
        cin >> col;

        cv::Point2d uv;

        uv.x = col;
        uv.y = row;

        std::cout<<"distance : "<<distance[row][col]<<std::endl;

        cv::Mat copy;
        copy = image.clone();

        while(true)
        {
            cv::circle(copy, uv, 3, cv::Scalar(int(255), int(0), int(0)), -1);
            cv::imshow("depth_viewer", copy);
            if(cv::waitKey(100) == 27)
                break;
        }

        std::string flag;
        std::cout<<" continue?(y/n) : ";
        cin >> flag;
        if(flag=="y" || flag=="Y")
            continue;
        else
            break;
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_viewer");

    Viewer ve;

    ve.main();

    return 0;
}
