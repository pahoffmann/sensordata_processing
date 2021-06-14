#include "kdTree.h"
#include <iostream> 
#include <random>
#include <fstream>
#include <algorithm>
#include <sstream>

/*
double **readPly() {

}
*/

int main (int argc, char **argv) {
    if(argc != 3)
    {
        std::cout << "insufficient number of input arguments" << std::endl;
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

    file.close();

    std::cout << "building kd-tree..." << std::endl;
    KDtree testTree(points, num_points);
    std::cout << "built kd-tree" << std::endl;

    return 0;
}