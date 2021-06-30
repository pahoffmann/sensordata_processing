#include <iostream>
#include <Eigen/Dense>
#include "kdTree.h"
#include <fstream>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <boost/program_options.hpp>
#include "culture_invariant_ply_writer.h"

/**
 * @brief prints the usage of the tree building stuff
 * 
 */
void printUsage()
{
    std::cout << "[filename][number of values per line]" << std::endl;
    std::cout << "For testmode just enter 'test' behind the executable" << std::endl;
}


int main (int argc, char **argv) {
    namespace po = boost::program_options;

    po::variables_map vm;
    po::options_description desc("Allowed Options");


    desc.add_options()
        ("help", "produce help message")
        ("filename", po::value<std::string>()->required(), "Type the name of the file which includes the points.")
        ("numargs", po::value<int>()->required(), "How many parameters are there each row?!")
        ("kn", po::value<int>()->required(), "How many neighbors do you wanna look at my m8?!")
        ("px", po::value<double>()->default_value(0.f), "orientation x")
        ("py", po::value<double>()->default_value(0.f), "orientation y")
        ("pz", po::value<double>()->default_value(0.f), "orientation z")
        ("numpoints", po::value<int>()->required(), "num points which will be evaluated")
        ("r2", po::value<double>()->default_value(0.1f), "squared radius");

    // parse arguments and save them in the variable map (vm)
    po::store(po::parse_command_line(argc, argv, desc), vm);
    

    std::ifstream file(vm["filename"].as<std::string>());
    
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
    file.open(vm["filename"].as<std::string>());

    int kn = vm["kn"].as<int>();
    double px = vm["px"].as<double>();
    double py = vm["py"].as<double>();
    double pz = vm["pz"].as<double>();

    int input_num_points = vm["numpoints"].as<int>();
    num_points = std::min(num_points, input_num_points);

    double x,y,z;
    int r, g, b, a;
    std::string line;

    double **points;
    points = new double*[num_points];


    std::vector<Eigen::Vector3d> vertex_normals;

    begin = std::chrono::steady_clock::now();

    for (int i = 0; i < num_points; i++) {
        
        //read data from stream
        if(vm["numargs"].as<int>() == 7)
        {
            if(!(file >> x >> y >> z >> r >> g >> b >> a))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(vm["numargs"].as<int>() == 6)
        {
            if(!(file >> x >> y >> z >> r >> g >> b))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(vm["numargs"].as<int>() == 3)
        {
            if(!(file >> x >> y >> z))
            {
                std::cout << "error when reading the data." << std::endl;
                break;
            }
        }
        else if(vm["numargs"].as<int>() == 5)
        {
            if(!(file >> x >> y >> z >> r >> g))
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

    end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    file.close();

    begin = std::chrono::steady_clock::now();
    std::cout << "building kd-tree..." << std::endl;
    KDtree myTree(points, num_points);
    std::cout << "built kd-tree" << std::endl;
    end = std::chrono::steady_clock::now();

    std::cout << "Time difference [Build] = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    double r2 = vm["r2"].as<double>();
        
    
    Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d centroid(0,0,0);


    std::cout << "Begin estimating normals" << std::endl;
    int counter = 0;
    int one_percent = num_points/100;


    // pca stuff
    begin = std::chrono::steady_clock::now();

    for (int i = 0; i < num_points; i++) {

        if(i % one_percent == 0)
        {
            std::cout << counter << " %" << std::endl;
            counter ++;
        }

        if(i % 1000 == 0)
        {
            end = std::chrono::steady_clock::now();
            std::cout << i << std::endl;
            auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            std::cout << "approx time left [s]: " << ((num_points - i) / 1000.0) * (diff/1000.0) << std::endl;
            begin = std::chrono::steady_clock::now();
        }
        // get knn of point
        int num_neighbors = 0;
        auto nNrs = myTree.kNearestNeighbors(points[i], r2, kn, num_neighbors);


        Eigen::Vector3d cur_vec(points[i][0],points[i][1],points[i][2]);
        centroid = Eigen::Vector3d::Zero();
        covariance_matrix = Eigen::Matrix3d::Zero();


        //calculate centroid
        for(int j = 0; j < num_neighbors; j++)
        {
            centroid += Eigen::Vector3d(nNrs[j][0],nNrs[j][1],nNrs[j][2]);            
        }

        centroid /= kn;

        for(int j = 0; j < num_neighbors; j++)
        {
            // calc current point moved by the clouds centroid
            Eigen::Vector3d cur_point(nNrs[j][0],nNrs[j][1],nNrs[j][2]);
            Eigen::Vector3d coVec = cur_point - centroid; // sub neighbors centroid
            covariance_matrix += coVec * coVec.transpose();

        }

        covariance_matrix /= num_neighbors; // fully calculated
        
        //std::cout << covariance_matrix << std::endl;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance_matrix);

        if (eigensolver.info() != Eigen::Success) 
        {
            continue;
        }
        
        auto eigen_values = eigensolver.eigenvalues();
        //std::cout << "Eigenvalues: " << std::endl << eigen_values << std::endl;
        auto eigen_vectors = eigensolver.eigenvectors();
        //std::cout << "Eigenvectors: " << std::endl << eigen_vectors << std::endl;

        int max_index;
        eigen_values.maxCoeff(&max_index);
        
        //std::cout << "Max coeff:" << max_index << std::endl;

        auto max_eigen_vec = eigen_vectors.col(0);
        
        // std::cout << "Max eigenvec: " << std::endl << max_eigen_vec << std::endl;

        // add normal to normal array
        Eigen::Vector3d normal_direction(px, py, pz);
        Eigen::Vector3d normal(max_eigen_vec.x(), max_eigen_vec.y(), max_eigen_vec.z());

        if(normal.dot(normal_direction - cur_vec) < 0)
        {

            normal *= -1;
        }

        vertex_normals.push_back(normal);

    }


    CultureInvariantPlyWriter writer("test.ply", points, vertex_normals);
    writer.Start();

    
    return 0;
}