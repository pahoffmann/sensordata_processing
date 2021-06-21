#ifndef __OCTREE__
#define __OCTREE__

#include <string>
#include <vector>
#include <Eigen/Dense>

class Octree {

    private:
        void _buildTree(std::vector<Eigen::Vector3d>);
        Octree(); // private constructor for building the tree
    public:
        double maxVoxSideLength;
        Eigen::Vector3d center;
        double sideLength;
        Octree *children[8];
        std::vector<Eigen::Vector3d> points;

        Octree(std::string filename); // private constructor for reeding the file
        ~Octree();

};

#endif