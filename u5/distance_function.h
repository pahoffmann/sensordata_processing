#ifndef __DISTANCE_FUNCTION__
#define __DISTANCE_FUNCTION__

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <flann/flann.hpp>

class DistanceFunction {
    private:
        flann::Index<flann::L2<double>> *index;
    public:
        float distance(float x, float y, float z, int k = 1);
        DistanceFunction(std::vector<Eigen::Vector3d> &points);
        ~DistanceFunction();
        DistanceFunction();
};
#endif