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
        std::vector<double> vertices;
        std::vector<int> indices;
        int nVert;
        int nFaces;
        char seperation = ',';
    public:
        CultureInvariantPlyWriter(std::string file, std::vector<double> &vertices, std::vector<int> &indices, int nVert, int nFaces);

        void Start();
};