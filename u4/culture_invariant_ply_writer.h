#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <numeric>
#include <vector>

class CultureInvariantPlyWriter
{
    private:
        std::string out_file_name;
        double **obj_points;
        std::vector<Eigen::Vector3d> normals;
        char seperation = ',';
    public:
        CultureInvariantPlyWriter(std::string file, double **input_pts, std::vector<Eigen::Vector3d> &input_normals);

        void Start();
};