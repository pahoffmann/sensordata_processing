#include "distance_function.h"


DistanceFunction::DistanceFunction(std::vector<Eigen::Vector3d> &points)
{
    // first: calculate normals

    
    // fill flann dataset 
    std::cout << points.size() << std::endl;
    
    flann::Matrix<double> dataset(new double[points.size() * 3], points.size(), 3);

    std::cout << "Initialized target data array" << std::endl;
    for(int i = 0; i < points.size(); i++)
    {
        dataset[i][0] = (points[i].x());
        dataset[i][1] = (points[i].y());
        dataset[i][2] = (points[i].z());
    }

    std::cout << "Initialized flann dataset" << std::endl;
    index = new flann::Index<flann::L2<double>>(dataset, flann::KDTreeSingleIndexParams(1, false));

    std::cout << "Initialized index..." << std::endl;

    // std::cout << __LINE__ << std::endl;

    index->buildIndex();

    std::cout << "Build flann index!" << std::endl;

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

    // double querydata[3];
    // querydata[0] = x;
    // querydata[1] = y;
    // querydata[2] = z;

    // flann::Matrix<double> query_point(querydata, 1, 3);

    // std::vector<std::vector<int>> indices;
    // std::vector<std::vector<double>> dists;

    // // calculate the KNNs
    // DistanceFunction::index->knnSearch(query_point, indices, dists, k, flann::SearchParams(128));

    // calculate the distance function from hoppe for each of those..

    return 0;
}
