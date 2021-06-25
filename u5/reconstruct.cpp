#include "octree.h"
#include <boost/program_options.hpp>
#include "culture_invariant_ply_writer.h"

int main(int argc, char **argv)
{
    namespace po = boost::program_options;

    po::variables_map vm;
    po::options_description desc("Allowed Options");


    desc.add_options()
        ("filename", po::value<std::string>()->required(), "Type the name of the file which includes the points.")
        ("numargs", po::value<int>()->required(), "How many parameters are there each row?!")
        ("kd", po::value<int>()->default_value(20), "How many neighbors do you wanna look at my m8?! [ distance ]")
        ("ki", po::value<int>()->default_value(10), "k nearest neighbors interpolation")
        ("kn", po::value<int>()->default_value(20), "k nearest neighbors normal estimation")
        ("vl", po::value<double>()->default_value(0.05), "max voxel length");

  // parse arguments and save them in the variable map (vm)
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // read params
    std::string filename = vm["filename"].as<std::string>();
    int numargs = vm["numargs"].as<int>();
    int kd = vm["kd"].as<int>();
    int ki = vm["ki"].as<int>();
    int kn = vm["kn"].as<int>();
    int vl = vm["vl"].as<double>();

    Octree octree(filename, numargs, vl, ki, kd, kn);
    double *vertices;
    int *indices;
    int nVert, nFaces;

    // reconstruct
    octree.reconstruct(vertices, indices, nVert, nFaces);

    //write to ply
    CultureInvariantPlyWriter writer("test.ply", vertices, indices, nVert, nFaces);
    writer.Start();
}