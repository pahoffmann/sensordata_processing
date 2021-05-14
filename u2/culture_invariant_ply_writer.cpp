#include "culture_invariant_ply_writer.h"


CultureInvariantPlyWriter::CultureInvariantPlyWriter(std::string file, std::vector<cv::Point3f>& input_pts)
{
    out_file_name = file;
    obj_points = input_pts;
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
    file << "element vertex " << obj_points.size() << std::endl;
    file << "property double x" << std::endl;
    file << "property double y" << std::endl;
    file << "property double z" << std::endl;
    file << "end_header" << std::endl;

    for(auto& point : obj_points)
    {   
        std::string x = std::to_string(point.x);
        std::string y = std::to_string(point.y);
        std::string z = std::to_string(point.z);

        if(seperation == ',')
        {
            std::replace(x.begin(), x.end(), '.', ',');
            std::replace(y.begin(), y.end(), '.', ',');
            std::replace(z.begin(), z.end(), '.', ',');
        }
        else
        {
            std::replace(x.begin(), x.end(), ',', '.');
            std::replace(y.begin(), y.end(), ',', '.');
            std::replace(z.begin(), z.end(), ',', '.');
        }

        file << x << " " << y << " " << z << std::endl;
    }

    file.close();
}
