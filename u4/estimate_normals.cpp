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
    namespace po = boost::program_options;

    po::variables_map vm;
    po::options_description desc("Allowed Options");


    desc.add_options()
        ("filename", po::value<std::string>()->required(), "Type the name of the file which includes the points.")
        ("numargs", po::value<int>()->required(), "How many parameters are there each row?!")
        ("kn", po::value<int>()->required(), "How many neighbors do you wanna look at my m8?!")
        ("px", po::value<double>()->default_value(0.f), "orientation x")
        ("py", po::value<double>()->default_value(0.f), "orientation y")
        ("pz", po::value<double>()->default_value(0.f), "orientation z");

  // parse arguments and save them in the variable map (vm)
    po::store(po::parse_command_line(argc, argv, desc), vm);
    
    /*
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

    */

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

    double r2 = 1.0f;
        
    
    Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d centroid(0,0,0);


    std::cout << "Begin estimating normals" << std::endl;
    int counter = 0;
    int one_percent = num_points/100;


    // pca stuff
    begin = std::chrono::steady_clock::now();

    for (int i = 0; i < 500; i++) {

        if(i % one_percent == 0)
        {
            std::cout << counter << " %" << std::endl;
            counter ++;
        }

        if(i % 100 == 0)
        {
            end = std::chrono::steady_clock::now();
            std::cout << i << std::endl;
            auto diff = std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
            std::cout << "approx time left [s]: " << ((num_points - i) / 100) * diff << std::endl;
            begin = std::chrono::steady_clock::now();
        }
        // get knn of point
        auto nNrs = myTree.kNearestNeighbors(points[i], r2, kn);

        Eigen::Vector3d cur_vec(nNrs[i][0],nNrs[i][1],nNrs[i][2]);
        centroid = Eigen::Vector3d::Zero();
        covariance_matrix = Eigen::Matrix3d::Zero();


        //calculate centroid
        for(int j = 0; j < kn; j++)
        {
            centroid += Eigen::Vector3d(nNrs[j][0],nNrs[j][1],nNrs[j][2]);            
        }

        centroid /= kn;

        for(int j = 0; j < kn; j++)
        {
            // calc current point moved by the clouds centroid
            Eigen::Vector3d cur_point(nNrs[j][0],nNrs[j][1],nNrs[j][2]);
            Eigen::Vector3d coVec = cur_point - centroid; // sub neighbors centroid
            covariance_matrix += coVec * coVec.transpose();

        }

        covariance_matrix /= kn; // fully calculated
        
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

        auto max_eigen_vec = eigen_vectors.col(max_index);
        
        // std::cout << "Max eigenvec: " << std::endl << max_eigen_vec << std::endl;

        // add normal to normal array
        vertex_normals.push_back(Eigen::Vector3d(max_eigen_vec.x(), max_eigen_vec.y(), max_eigen_vec.z()));
    }


    CultureInvariantPlyWriter writer("test.ply", points, vertex_normals);
    writer.Start();


    
    return 0;
}