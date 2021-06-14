#include "culture_invariant_ply_reader.h"


CultureInvariantPlyReader::CultureInvariantPlyReader(std::string file)
{
    in_file_name = file;
    auto locale_ = std::locale("").name();
    if (locale_.find("de_DE.UTF-8") != std::string::npos)
    {
        seperation = '.';
    }
}

Status CultureInvariantPlyReader::Start(std::vector<cv::Point3d>& output_arr)
{
    std::ifstream file(in_file_name);
    output_arr.clear();
    

    if(!file.is_open())
    {
        return BAD_FILE_READ;
    }

    std::string type, format, line;
    int num_vertices;
    
    std::getline(file, type); 
    std::getline(file, format); 

    // we only support ascii ply files
    if(type != "ply" || format.find("ascii") == std::string::npos)
    {
        return WRONG_FILE_FORMAT;
    }

    //also we only support 3d pointclouds, because 
    while (std::getline(file, line))
    {
        if(line.find("comment") != std::string::npos)
        {
            std::cout << line << std::endl;
        }

        if(line.find("element vertex") != std::string::npos)
        {
            num_vertices = std::stoi(line.substr(1,line.find("vertex ")));

            std::cout << "num vertices: " << num_vertices << std::endl;
        }

        //header read
        if(line.find("end_header") != std::string::npos)
        {
            break;
        }
    }

    double x,y,z;

    for(int i = 0; i < num_vertices; i++)
    {
        file >> x >> y >> z;
        std::cout << "added point : " << x << " | " << y << " | " << z << std::endl;
        output_arr.push_back(cv::Point3d(x,y,z));
    }

    file.close();
}
