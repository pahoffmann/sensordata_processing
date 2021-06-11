#include "kdTree.h"
#include <iostream> 
#include <random>

/*
double **readPly() {

}
*/

int main (int argc, char **argv) {
    
    double **points;
    int npts = 20;
    points = new double*[npts];
    double MIN = -10;
    double MAX = 10;
    std::uniform_real_distribution<double> unif(MIN, MAX);
    std::default_random_engine re;

    for (int i = 0; i < npts; i++) {
        points[i] = new double[3];
        points[i][0] = unif(re);
        points[i][1] = unif(re);
        points[i][2] = unif(re);

    }

    KDtree testTree(points, npts);
    std::cout << "built kd-tree" << std::endl;

    return 0;
}