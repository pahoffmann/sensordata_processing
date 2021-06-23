#include "octree.h"

int main(int argc, char** argv)
{
    if(argc == 2)
    {
        Octree octree("panagia_kera_interior.txt", 7, std::atof(argv[1]));  
    }
    else {
        Octree octree("panagia_kera_interior.txt", 7, 0.1);
    }
}