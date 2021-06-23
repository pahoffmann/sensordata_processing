#include "distance_function.h"


DistanceFunction::DistanceFunction(std::vector<Eigen::Vector3d> &points)
{
    // first: calculate normals

    
    // fill flann dataset 
    std::cout << points.size() << std::endl;
    
    float targetData[(points.size() * 3)]; // holds data

    std::cout << "Initialized target data array" << std::endl;
    for(int i = 0; i < points.size(); i++)
    {
        targetData[i * 3 + 0] = (float)(points[i].x());
        targetData[i * 3 + 1] = (float)(points[i].y());
        targetData[i * 3 + 2] = (float)(points[i].z());
    }

    // flann::Matrix<float> dataset(targetData, points.size(), 3);

    // std::cout << "Initialized flann dataset" << std::endl;
    // index = new flann::Index<flann::L2<float>>(dataset, flann::KDTreeIndexParams(1));

    // std::cout << "Initialized index..." << std::endl;

    // // std::cout << __LINE__ << std::endl;

    // index->buildIndex();

    // std::cout << "Build flann index!" << std::endl;

}

DistanceFunction::~DistanceFunction()
{
    // todo
}

DistanceFunction::DistanceFunction()
{
    // emtpy default constructor
}


float DistanceFunction::distance(float x, float y, float z, int k)
{
    // find nearest neighbor (k nearst), go through each of them, calculate distance via (-1|+1) * (p-c)*n

    float querydata[3];
    querydata[0] = x;
    querydata[1] = y;
    querydata[2] = z;

    flann::Matrix<float> query_point(querydata, 1, 3);

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;

    // calculate the KNNs
    DistanceFunction::index->knnSearch(query_point, indices, dists, k, flann::SearchParams(128));

    // calculate the distance function from hoppe for each of those..

    return 0;
}
