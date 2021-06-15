#include "kdTree.h"
#include <iostream> 
#include <random>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <flann/flann.h>

/*
double **readPly() {

}
*/

/**
 * @brief prints the usage of the tree building stuff
 * 
 */
void printUsage()
{
    std::cout << "[filename][number of values per line]" << std::endl;
    std::cout << "For testmode just enter 'test' behind the executable" << std::endl;
}

/**
 * @brief creates a cube between (0,0,0) and (1,1,1) and creates a kd tree from its points. checks if the points are saved correctly
 * 
 */
void testTree()
{
    double **points;
    points = new double*[8];
    int i = 0;
    
    for(int x = 0; x <= 1; x++)
    {
        for(int y = 0; y <= 1; y++)
        {
            for(int z = 0; z <= 1; z++)
            {

                points[i] = new double[3];
                points[i][0] = (double)x;
                points[i][1] = (double)y;
                points[i][2] = (double)z;

                i++;     
            }
        }
    }

    KDtree testTree(points, 8);
    //std::cout << testTree.node.child1->node.child1->leaf.p[0][0]<< " | " << testTree.node.child1->node.child1->leaf.p[0][1] << " | " << testTree.node.child1->node.child1->leaf.p[0][1] << std::endl;
    //std::cout << testTree.node.child2->node.child2->leaf.p[1][0]<< " | " << testTree.node.child2->node.child2->leaf.p[1][1] << " | " << testTree.node.child2->node.child2->leaf.p[1][1] << std::endl;
    double testPoint[3] =  {0.0, 0.0, 0.5};
    auto nN = testTree.FindClosest(testPoint, 0.8);
    if(nN != NULL)
    {
        std::cout << "Closest: " << nN[0] << " | " << nN[1] << " | " << nN[2] << std::endl; 
    }

    auto nNrs = testTree.kNearestNeighbors(testPoint, 2.5, 4);
    for(int i = 0; i < 4; i++)
    {
        if(nNrs[i]!= NULL)
        {
            std::cout << nNrs[i][0] << " | " << nNrs[i][1] << " | " << nNrs[i][2] << std::endl;
        }
    }
}

int main (int argc, char **argv) {

    std::string input(argv[1]);

    if(input.find("test") != std::string::npos)
    {

        std::cout << "Using test mode" << std::endl;
        testTree();
        return EXIT_SUCCESS;
    }


    if(argc != 3)
    {
        std::cout << "insufficient number of input arguments" << std::endl;
        printUsage();

        return EXIT_FAILURE;
    }

    std::string input_file_name = argv[1];
    int num_arguments = atoi(argv[2]);
    std::cout << "Input file name: " << input_file_name << std::endl;
    std::cout << "Input file arguments: " << num_arguments << std::endl; 
    std::ifstream file(input_file_name);
    
    // check ig the file could be opened
    if(!file.is_open())
    {
        std::cout << "could not open file" << std::endl;
        printUsage();
        return EXIT_FAILURE;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    //count lines
    int num_points = std::count(std::istreambuf_iterator<char>(file), 
             std::istreambuf_iterator<char>(), '\n');

    std::cout << "Num Points: " << num_points << std::endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference [Line Count] = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;


    file.close();
    file.open(input_file_name);

    double x,y,z;
    int r, g, b, a;
    std::string line;

    double **points;
    points = new double*[num_points];
    // flann data representation
    double *dataset;
    dataset = new double[num_points * 3];

    begin = std::chrono::steady_clock::now();


    for (int i = 0; i < num_points; i++) {
        
        //read data from stream
        if(num_arguments == 7)
        {
            if(!(file >> x >> y >> z >> r >> g >> b >> a))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(num_arguments == 6)
        {
            if(!(file >> x >> y >> z >> r >> g >> b))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }

        //std::cout << "Point: " << x << " | " << y << " | " << z << std::endl;;

        points[i] = new double[3];
        points[i][0] = x;
        points[i][1] = y;
        points[i][2] = z;
        dataset[i*3] = x;
        dataset[i*3+1] = y;
        dataset[i*3+2] = z;

    }

    std::cout << "finished reading the points " << std::endl;

    end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    file.close();

    begin = std::chrono::steady_clock::now();
    std::cout << "building kd-tree..." << std::endl;
    KDtree testTree(points, num_points);
    std::cout << "built kd-tree" << std::endl;
    end = std::chrono::steady_clock::now();

    std::cout << "Time difference [Build] = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    double testVec[3] = {0,0,0};
    int nn = 5;        // number of nearest neighbors
    double r2 = 3.f;     // search radius
    begin = std::chrono::steady_clock::now();

    auto nNrs = testTree.kNearestNeighbors(testVec, r2, nn);
    end = std::chrono::steady_clock::now();

    std::cout << "Time difference [Search] = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    for(int i = 0; i < nn; i++)
    {
        if(nNrs[i]!= NULL)
        {
            std::cout << nNrs[i][0] << " | " << nNrs[i][1] << " | " << nNrs[i][2] << std::endl;
        }
    }


    std::cout << "Testing flann implementation" << std::endl;
    struct FLANNParameters p;
    int *result;
    double* dists;
    flann_index_t index_id;
    float speedup;

    result = (int*) malloc(nn*num_points*sizeof(int));
    dists = (double*) malloc(nn*num_points*sizeof(double));
    p = DEFAULT_FLANN_PARAMETERS;
    p.algorithm = FLANN_INDEX_KDTREE;
    p.trees = 1;
    p.log_level = FLANN_LOG_INFO;
    p.checks = 256;
    p.target_precision = 1.f;
    p.sorted = 1;

    std::cout << "building index" << std::endl;
    index_id = flann_build_index_double(dataset, num_points, 3, &speedup, &p);

    std::cout << "searching nearest neigbors with flann" << std::endl;
    begin = std::chrono::steady_clock::now();
    flann_radius_search_double(index_id, testVec, result, dists, num_points, r2, &p);
    end = std::chrono::steady_clock::now();
    std::cout << "Time difference [Search] = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    std::cout << "DEBUG: " << result[0] << ", " << result[1] << ", " << result[2] << std::endl;
    std::cout << "DEBUG: " << result[3] << ", " << result[4] << ", " << result[5] << std::endl;

    for(int i = 0; i < nn; i++)
    {
        std::cout << dataset[result[i]] << " | " << dataset[result[i]+1] << " | " << dataset[result[i]+2] << std::endl;
        }

    return 0;
}