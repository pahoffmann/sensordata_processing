#include "culture_invariant_ply_writer.h"


CultureInvariantPlyWriter::CultureInvariantPlyWriter(std::string file, double **input_pts, std::vector<Eigen::Vector3d> &input_normals)
{
    out_file_name = file;
    obj_points = input_pts;
    normals = input_normals;
    auto locale_ = std::locale("").name();
    if (locale_.find("de_DE.UTF-8") != std::string::npos)
    {
        seperation = '.';
    }
}

void CultureInvariantPlyWriter::Start()
{
    std::ofstream file(out_file_name);

    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "comment made by patoriko hoffmann" << std::endl;
    file << "element vertex " << normals.size() << std::endl;
    file << "property double x" << std::endl;
    file << "property double y" << std::endl;
    file << "property double z" << std::endl;
    file << "property double nx" << std::endl;
    file << "property double ny" << std::endl;
    file << "property double nz" << std::endl;
    file << "end_header" << std::endl;

    for(int i = 0; i < normals.size(); i++)
    {   
        std::string x = std::to_string(obj_points[i][0]);
        std::string y = std::to_string(obj_points[i][1]);
        std::string z = std::to_string(obj_points[i][2]);
        std::string nx = std::to_string(normals[i].x());
        std::string ny = std::to_string(normals[i].y());
        std::string nz = std::to_string(normals[i].z());

        if(seperation == ',')
        {
            std::replace(x.begin(), x.end(), '.', ',');
            std::replace(y.begin(), y.end(), '.', ',');
            std::replace(z.begin(), z.end(), '.', ',');
            std::replace(nx.begin(), nx.end(), '.', ',');
            std::replace(ny.begin(), ny.end(), '.', ',');
            std::replace(nz.begin(), nz.end(), '.', ',');
        }
        else
        {
            std::replace(x.begin(), x.end(), ',', '.');
            std::replace(y.begin(), y.end(), ',', '.');
            std::replace(z.begin(), z.end(), ',', '.');
            std::replace(nx.begin(), nx.end(), ',', '.');
            std::replace(ny.begin(), ny.end(), ',', '.');
            std::replace(nz.begin(), nz.end(), ',', '.');
        }

        file << x << " " << y << " " << z << " " << nx << " " << ny << " " << nz << std::endl;
    }

    file.close();
}
