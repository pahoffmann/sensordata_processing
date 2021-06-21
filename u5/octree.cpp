#include "octree.h"


// internal private constructor, used to build the octree
Octree::Octree(std::vector<Eigen::Vector3d> points)
{

}

/**
 * @brief Construct a new Octree:: Octree object
 *  public constructor used to build the tree, reads the transmitted filename!
 * 
 * @param filename   name of the file containing the point information
 */
Octree::Octree(std::string filename, int num_args = 3)
{
     std::ifstream file(filename);
    
    // check ig the file could be opened
    if(!file.is_open())
    {
        std::cout << "could not open file" << std::endl;
        return;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    double x,y,z;
    int r, g, b, a;
    std::string line;
    std::vector<Eigen::Vector3d> input_points;

    begin = std::chrono::steady_clock::now();

    for (;;) {
        
        //read data from stream
        if(num_args == 7)
        {
            if(!(file >> x >> y >> z >> r >> g >> b >> a))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(num_args == 6)
        {
            if(!(file >> x >> y >> z >> r >> g >> b))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(num_args == 3)
        {
            if(!(file >> x >> y >> z))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(num_args == 5)
        {
            if(!(file >> x >> y >> z >> r >> g))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }

        //std::cout << "Point: " << x << " | " << y << " | " << z << std::endl;;

        input_points.push_back(Eigen::Vector3d(x,y,z));
    }


    std::cout << "finished reading the points. Num points:  "<< input_points.size() << std::endl;

    end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    file.close();

    std::cout << "Start iterating through the box to determine the needed parameters." << std::endl;
    begin = std::chrono::steady_clock::now();

    double max_x, max_y, max_z, min_x, min_y, min_z;
    max_x = max_y = max_z = std::numeric_limits<double>::min();
    min_x = min_y = min_z = std::numeric_limits<double>::max();

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for(auto point : input_points)
    {
        // bounding box
        if(point.x() < min_x) min_x = point.x();
        if(point.y() < min_y) min_y = point.y();
        if(point.z() < min_z) min_z = point.z();
        if(point.x() > max_x) max_x = point.x();
        if(point.y() > max_y) max_y = point.y();
        if(point.z() > max_z) max_z = point.z();

        centroid += point;
        //std::cout << centroid << std::endl;
    }

    centroid /= input_points.size(); // calc centroid

    end = std::chrono::steady_clock::now();

    std::cout << "Finished itering " << points.size() << " points. Centroid is: " << std::endl << centroid << std::endl;
    std::cout << "Used time = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    if(points.size() > 0)
    {
        //create new subtrees

    }

    // iterate through the points once, capture bounding box, center, sidelengths etc.

}

Octree::~Octree()
{
    // empty
}