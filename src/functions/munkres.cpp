#include <ros/ros.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

using namespace std;

double min (const double& a, const double& b)
{
	return a < b ? a : b;
}

double max (const double& a, const double& b)
{
	return a > b ? a : b;
}


void hungarian(Eigen::MatrixXi M, std::vector<int>& data)
{
	const int Inf = 1e6;

	int n = M.rows(), p, q;
	vector<int> fx(n, Inf), fy(n, 0);
	vector<int> x(n, -1), y(n, -1);

	for(int i = 0; i < n; ++i){
		for(int j = 0; j < n; ++j){
			fx[i] = min(fx[i], M(i, j));
		}
	}

	for(int i = 0; i < n; ){
		vector<int> t(n, -1), s(n+1, i);
		for(p = q = 0; p <= q && x[i] < 0; ++p){
			for(int k = s[p], j = 0; j < n && x[i] < 0; ++j){
				if (fx[k] + fy[j] == M(k, j) && t[j] < 0){
					s[++q] = y[j], t[j] = k;
					if(s[q] < 0){
						for(p = j; p >= 0; j = p){
							y[j] = k = t[j], p = x[k], x[k] = j;
						}
					}
				}
			}
		}
		if(x[i] < 0){
			int d = Inf;
			for(int k = 0; k <= q; ++k){
				for(int j = 0; j < n; ++j){
					if(t[j] < 0) d = max(d, fx[s[k]] + fy[j] - M(s[k], j));
				}
			}
			for(int j = 0; j < n; ++j) fy[j] += (t[j] < 0 ? 0 : d);
			for(int k = 0; k <= q; ++k) fx[s[k]] -= d;
		}else ++i;
	}

    for(size_t i=0;i<y.size();i++)
        data.push_back(y[i]);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "munkres");

    int size = 4;
    Eigen::MatrixXi M(size, size);

    M << 5, 4, 7, 6,
         6, 7, 3, 2, 
         8,11, 2, 5, 
         9, 8, 6, 7;
    

    std::cout<<M<<std::endl;

    std::vector<int> data;
    hungarian(M, data);

    for(size_t i=0;i<data.size();i++)
        std::cout<<"("<<i<<" "<<data[i]<<")"<<std::endl;


    return 0;


}
