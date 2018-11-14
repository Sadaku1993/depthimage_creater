#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename T_p, typename T_c, typename T_ptr>
class QuickSort{
    private:

    public:
        double distance(T_p point);
        void quicksort(double numbers[], int left, int right);
        void pcl_quicksort(T_c& cloud, int left, int right);
        void pcl_sort(T_c& cloud);
        int min(double *all);
        void show_cloud(T_c cloud);
        void split_sort(T_c cloud, T_c& result);
};

template<typename T_p, typename T_c, typename T_ptr>
double QuickSort<T_p, T_c, T_ptr>::distance(T_p point)
{
    double dis = sqrt( pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0));
    return dis;
}

template<typename T_p, typename T_c, typename T_ptr>
void QuickSort<T_p, T_c, T_ptr>::quicksort(double numbers[], int left, int right)
{
    int l_hold, r_hold;
    double pivot;

    l_hold = left;
    r_hold = right;
    pivot = numbers[left];
    while (left < right)
    {
        while ((numbers[right] >= pivot) && (left < right))
            right--;
        if (left != right)
        {
            numbers[left] = numbers[right];
            left++;
        }
        while ((numbers[left] <= pivot) && (left < right))
            left++;
        if (left != right)
        {
            numbers[right] = numbers[left];
            right--;
        }
    }
    numbers[left] = pivot;
    pivot = left;
    left = l_hold;
    right = r_hold;
    if (left < pivot)
        quicksort(numbers, left, pivot-1);
    if (right > pivot)
        quicksort(numbers, pivot+1, right);
}

template<typename T_p, typename T_c, typename T_ptr>
void QuickSort<T_p, T_c, T_ptr>::pcl_quicksort(T_c &cloud, int left, int right)
{
    int l_hold, r_hold;
    double pivot;
    T_p tmp;

    l_hold = left;
    r_hold = right;
    pivot = distance(cloud.points[left]);

    tmp = cloud.points[left];

    while (left < right)
    {
        while ((distance(cloud.points[right]) >= pivot) && (left < right))
            right--;
        if (left != right)
        {
            cloud.points[left] = cloud.points[right];
            left++;
        }
        while ((distance(cloud.points[left]) <= pivot) && (left < right))
            left++;
        if (left != right)
        {
            cloud.points[right] = cloud.points[left];
            right--;
        }
    }
    cloud.points[left] = tmp;
    
    if(l_hold<left)
        pcl_quicksort(cloud, l_hold, left-1);
    if(r_hold > left)
        pcl_quicksort(cloud, left+1, r_hold);
}

template<typename T_p, typename T_c, typename T_ptr>
void QuickSort<T_p, T_c, T_ptr>::pcl_sort(T_c &cloud)
{
    for(size_t i=0;i<cloud.points.size();i++){
        for(size_t j=i+1;j<cloud.points.size();j++){
            T_p p_i = cloud.points[i];
            T_p p_j = cloud.points[j];
            double dis_i = sqrt( pow(p_i.x, 2.0) + pow(p_i.y, 2.0) + pow(p_i.z, 2.0));
            double dis_j = sqrt( pow(p_j.x, 2.0) + pow(p_j.y, 2.0) + pow(p_j.z, 2.0));
            if(dis_i > dis_j){
                T_p tmp = cloud.points[i];
                cloud.points[i] = cloud.points[j];
                cloud.points[j] = tmp;
            }
        }
    }
}

template<typename T_p, typename T_c, typename T_ptr>
int QuickSort<T_p, T_c, T_ptr>::min(double *all)
{
    int num = 0;
    double min=all[0];
    for(int i=0;i<6;i++){
        if(min>all[i]){
            min=all[i];
            num=i;
        }
    }
    return num+1;
}

template<typename T_p, typename T_c, typename T_ptr>
void QuickSort<T_p, T_c, T_ptr>::show_cloud(T_c cloud)
{
    for(size_t i=0;i<cloud.points.size();i++)
    {
        double dis = distance(cloud.points[i]);
        printf("No:%5d x:%6.2f y:%6.2f z:%6.2f distance:%6.2f\n",
                int(i), cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, dis);
    }
}
 
template<typename T_p, typename T_c, typename T_ptr>
void QuickSort<T_p, T_c, T_ptr>::split_sort(T_c cloud, T_c &result)
{
    T_ptr cloud1(new T_c);
    T_ptr cloud2(new T_c);
    T_ptr cloud3(new T_c);
    T_ptr cloud4(new T_c);
    T_ptr cloud5(new T_c);
    T_ptr cloud6(new T_c);

    size_t size = cloud.points.size();
    size_t size_1 = size/6;
    size_t size_2 = size/6;
    size_t size_3 = size/6;
    size_t size_4 = size/6;
    size_t size_5 = size/6;
    size_t size_6 = size - size/6*5;

    cloud1->points.resize(size_1);
    cloud2->points.resize(size_2);
    cloud3->points.resize(size_3);
    cloud4->points.resize(size_4);
    cloud5->points.resize(size_5);
    cloud6->points.resize(size_6);

    #pragma omp parallel
    {
        for(size_t i=0;i<size_1;i++)
            cloud1->points[i] = cloud.points[i];
        for(size_t i=0;i<size_2;i++)
            cloud2->points[i] = cloud.points[i+size_1];
        for(size_t i=0;i<size_3;i++)
            cloud3->points[i] = cloud.points[i+size_1+size_2];
        for(size_t i=0;i<size_4;i++)
            cloud4->points[i] = cloud.points[i+size_1+size_2+size_3];
        for(size_t i=0;i<size_5;i++)
            cloud5->points[i] = cloud.points[i+size_1+size_2+size_3+size_4];
        for(size_t i=0;i<size_6;i++)
            cloud6->points[i] = cloud.points[i+size_1+size_2+size_3+size_4+size_5];
    }

    #pragma omp parallel
    {
        if(omp_get_thread_num() == 1)
            pcl_quicksort(*cloud1, 0, int(cloud1->points.size()-1));
        else if(omp_get_thread_num() == 2)
            pcl_quicksort(*cloud2, 0, int(cloud2->points.size()-1));
        else if(omp_get_thread_num() == 3)
            pcl_quicksort(*cloud3, 0, int(cloud3->points.size()-1));
        else if(omp_get_thread_num() == 4)
            pcl_quicksort(*cloud4, 0, int(cloud4->points.size()-1));
        else if(omp_get_thread_num() == 5)
            pcl_quicksort(*cloud5, 0, int(cloud5->points.size()-1));
        else if(omp_get_thread_num() == 6)
            pcl_quicksort(*cloud6, 0, int(cloud6->points.size()-1));
    }

    size_t count1 = 0;
    size_t count2 = 0;
    size_t count3 = 0;
    size_t count4 = 0;
    size_t count5 = 0;
    size_t count6 = 0;

    for(size_t i=0;i<cloud.points.size();i++)
    {
        double dis1 = distance(cloud1->points[count1]);
        double dis2 = distance(cloud2->points[count2]);
        double dis3 = distance(cloud3->points[count3]);
        double dis4 = distance(cloud4->points[count4]);
        double dis5 = distance(cloud5->points[count5]);
        double dis6 = distance(cloud6->points[count6]);

        if(int(count1)==int(cloud1->points.size())) dis1=INFINITY;
        if(int(count2)==int(cloud2->points.size())) dis2=INFINITY;
        if(int(count3)==int(cloud3->points.size())) dis3=INFINITY;
        if(int(count4)==int(cloud4->points.size())) dis4=INFINITY;
        if(int(count5)==int(cloud5->points.size())) dis5=INFINITY;
        if(int(count6)==int(cloud6->points.size())) dis6=INFINITY;

        double array[]={dis1, dis2, dis3, dis4, dis5, dis6};

        int num = min(array);

        if(num==1){
            result.points.push_back(cloud1->points[count1]);
            count1++;
        }else if(num==2){
            result.points.push_back(cloud2->points[count2]);
            count2++;
        }else if(num==3){
            result.points.push_back(cloud3->points[count3]);
            count3++;
        }else if(num==4){
            result.points.push_back(cloud4->points[count4]);
            count4++;
        }else if(num==5){
            result.points.push_back(cloud5->points[count5]);
            count5++;
        }else if(num==6){
            result.points.push_back(cloud6->points[count6]);
            count6++;
        }
    }
}
