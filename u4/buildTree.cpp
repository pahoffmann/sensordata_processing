#include "kdTree.h"
#include <iostream> 
#include <random>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <chrono>

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

    //count lines
    int num_points = std::count(std::istreambuf_iterator<char>(file), 
             std::istreambuf_iterator<char>(), '\n');

    std::cout << "Num Points: " << num_points << std::endl;

    file.close();
    file.open(input_file_name);

    double x,y,z;
    int r, g, b, a;
    std::string line;

    double **points;
    points = new double*[num_points];

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


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

    }

    std::cout << "finished reading the points " << std::endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    file.close();

    begin = std::chrono::steady_clock::now();
    std::cout << "building kd-tree..." << std::endl;
    KDtree testTree(points, num_points);
    std::cout << "built kd-tree" << std::endl;
    end = std::chrono::steady_clock::now();

    std::cout << "Time difference [Build] = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return 0;
}