#include "culture_invariant_ply_writer.h"


CultureInvariantPlyWriter::CultureInvariantPlyWriter(std::string file, double *vertices, int *indices, int nVert, int nFaces)
{
    out_file_name = file;
    this->vertices = vertices;
    this->indices = indices;
    this->nVert = nVert;
    this->nFaces = nFaces;
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
    file << "comment made by joe mama" << std::endl;
    file << "element vertex " << nVert << std::endl;
    file << "property double x" << std::endl;
    file << "property double y" << std::endl;
    file << "property double z" << std::endl;
    file << "element face " << nFaces << std::endl;
    file << "property list uchar int vertex_indices" << std::endl;
    file << "end_header" << std::endl;

    for(int i = 0; i < nVert; i+=3)
    {   
        std::string x = std::to_string(vertices[i]);
        std::string y = std::to_string(vertices[i+1]);
        std::string z = std::to_string(vertices[i+2]);

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

    for (int i = 0; i < nFaces; i+=3) {
        file << indices[i] << indices[i+1] << indices[i+2] << std::endl;
    }

    file.close();
}
