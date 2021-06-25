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
        flann::Matrix<double> dataset_;
        int ki;
        int kd;
        int kn;
    public:
        double distance(double x, double y, double z, int k = 1);
        DistanceFunction(std::vector<Eigen::Vector3d> &points, int ki, int kd, int kn);
        ~DistanceFunction();
        DistanceFunction();
};
#endif