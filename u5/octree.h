#ifndef __OCTREE__
#define __OCTREE__

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>

class Octree {

    private:
        static double maxVoxSideLength;

        Octree(std::vector<Eigen::Vector3d*> &points, double prev_sidelengths[3], Eigen::Vector3d &prev_centroid); // private constructor for building the tree
    public:
        Eigen::Vector3d center;
        double sideLength[3]; // contains the side lengths of the current subtree in each of the 3 directions
        Octree *children[8]; // every node contains 8 leafs
        std::vector<Eigen::Vector3d> points; // only filled, if it is a leaf
        bool is_leaf = false; // determines if the current node is a leaf (e.g. it has no further subtrees)

        Octree(std::string filename, int num_args = 7, double maxSideLength = -1); // private constructor for reeding the file
        ~Octree();

};

#endif