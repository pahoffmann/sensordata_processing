#include "octree.h"

double Octree::maxVoxSideLength;
std::vector<Eigen::Vector3d> Octree::point_buf;
int Octree::num_leafpoints_acc;
int Octree::num_leafs;

// internal private constructor, used to build the octree
Octree::Octree(std::vector<Eigen::Vector3d*> &points, double new_sidelengths[3], Eigen::Vector3d &new_centroid)
{
    // first calc new values:
    this->sideLength[0] = new_sidelengths[0];
    this->sideLength[1] = new_sidelengths[1];
    this->sideLength[2] = new_sidelengths[2];

    // new centroid
    this->center = new_centroid;
    
    double len_x = this->sideLength[0];
    double len_y = this->sideLength[1];
    double len_z = this->sideLength[2];

    // if it is a leaf 
    if(std::max(std::max(len_x, len_y), len_z) < maxVoxSideLength)
    {
        Octree::num_leafs++;
        Octree::num_leafpoints_acc += (int)points.size();

        for(auto &p: points)
        {
            this->points.push_back(*p);
        }

        this->is_leaf = true;

        return;
    }

    std::vector<Eigen::Vector3d*> tlf, trf, tlb, trb, blf, brf, blb, brb; // pointers for 8 subtrees

    
    if(points.size() > 0)
    {
        // create new octrees
        // determine points which need to be passed on to the children
        // determine center points, sidelengths etc.
        for(auto &p : points)
        {   
            // bottom [b]
            if(p->y() < this->center.y())
            {
                if(p->x() < this->center.x()) // left [l]
                {
                    if(p->z() < this->center.z()) // back [b]
                    {
                        blb.push_back(p);
                    }
                    else // front [f]
                    {
                        blf.push_back(p);
                    }
                }
                else // right [r]
                {
                    if(p->z() < this->center.z()) // back[b]
                    {
                        brb.push_back(p);
                    }
                    else //front[f]
                    {
                        brf.push_back(p);
                    }
                }
            }
            else // top [t]
            {
                if(p->x() < this->center.x()) // left [l]
                {
                    if(p->z() < this->center.z()) // back [b]
                    {
                        tlb.push_back(p);
                    }
                    else // front [f]
                    {
                        tlf.push_back(p);
                    }
                }
                else // right [r]
                {
                    if(p->z() < this->center.z()) // back[b]
                    {
                        trb.push_back(p);
                    }
                    else //front[f]
                    {  
                        trf.push_back(p);
                    }
                }
            }
            
        }

        // std::cout << "Subtree point sizes: " << std::endl
        //           << blb.size() << std::endl
        //           << blf.size() << std::endl
        //           << brb.size() << std::endl
        //           << brf.size() << std::endl
        //           << tlb.size() << std::endl
        //           << tlf.size() << std::endl
        //           << trb.size() << std::endl
        //           << trf.size() << std::endl;

        //top left front to bottom right back


        double new_sidelength[3] = {len_x / 2, len_y / 2, len_z / 2};

        // calculate new centroids
        Eigen::Vector3d tlf_a = this->center + Eigen::Vector3d(-len_x / 2, len_y/2, len_z /2);
        Eigen::Vector3d trf_a = this->center + Eigen::Vector3d(len_x / 2, len_y/2, len_z /2);
        Eigen::Vector3d tlb_a = this->center + Eigen::Vector3d(-len_x / 2, len_y/2, -len_z /2);
        Eigen::Vector3d trb_a = this->center + Eigen::Vector3d(len_x / 2, len_y/2, -len_z /2);
        Eigen::Vector3d blf_a = this->center + Eigen::Vector3d(-len_x / 2, -len_y/2, len_z /2);
        Eigen::Vector3d brf_a = this->center + Eigen::Vector3d(len_x / 2, -len_y/2, len_z /2);
        Eigen::Vector3d blb_a = this->center + Eigen::Vector3d(-len_x / 2, -len_y/2, -len_z /2);
        Eigen::Vector3d brb_a = this->center + Eigen::Vector3d(len_x / 2, -len_y/2, -len_z /2);

        this->children[0] = new Octree(tlf, new_sidelength, tlf_a);
        this->children[1] = new Octree(trf, new_sidelength, trf_a);
        this->children[2] = new Octree(tlb, new_sidelength, tlb_a);
        this->children[3] = new Octree(trb, new_sidelength, trb_a);
        this->children[4] = new Octree(blf, new_sidelength, blf_a);
        this->children[5] = new Octree(brf, new_sidelength, brf_a);
        this->children[6] = new Octree(blb, new_sidelength, blb_a);
        this->children[7] = new Octree(brb, new_sidelength, brb_a);
    }
}

/**
 * @brief Construct a new Octree:: Octree object
 *  public constructor used to build the tree, reads the transmitted filename!
 * 
 * @param filename   name of the file containing the point information
 */
Octree::Octree(std::string filename, int num_args, double maxSideLength)
{
    maxVoxSideLength = maxSideLength;



     std::ifstream file(filename);
    
    // check ig the file could be opened
    if(!file.is_open())
    {
        std::cout << "could not open file" << std::endl;
        return;
    }

    double x,y,z;
    int r, g, b, a;
    std::string line;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


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

        point_buf.push_back(Eigen::Vector3d(x,y,z));
    }


    std::cout << "finished reading the points. Num points:  "<< point_buf.size() << std::endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    file.close();

    std::cout << "Start iterating through the box to determine the needed parameters." << std::endl;
    begin = std::chrono::steady_clock::now();

    double max_x, max_y, max_z, min_x, min_y, min_z;
    max_x = max_y = max_z = std::numeric_limits<double>::min();
    min_x = min_y = min_z = std::numeric_limits<double>::max();

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for(auto &p : Octree::point_buf)
    {
        // bounding box

        double x = p.x();
        double y = p.y();
        double z = p.z();

        if(x < min_x) min_x = x;
        if(y < min_y) min_y = y;
        if(z < min_z) min_z = z;
        if(x > max_x) max_x = x;
        if(y > max_y) max_y = y;
        if(z > max_z) max_z = z;

        centroid += Eigen::Vector3d(x, y, z);
        //std::cout << centroid << std::endl;
    }

    centroid /= point_buf.size(); // calc centroid

    end = std::chrono::steady_clock::now();

 
    std::cout << "Finished itering " << point_buf.size() << " points. Centroid is: " << std::endl << centroid << std::endl;
    std::cout << "Used time for O(n) iteration = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    double len_x = max_x - min_x;
    double len_y = max_y - min_y;
    double len_z = max_z - min_z;

    std::cout << "Initial Bounding Box side-lengths: " << len_x << " | " << len_y << " | " << len_z << std::endl;

    // error handling, determine max distance manually...
    if(maxVoxSideLength == -1 || maxVoxSideLength <= 0)
    {
        double max = std::max(len_x, len_y);
        max = std::max(max, len_z);

        std::cout << "Max side length: " << max << std::endl;
        maxVoxSideLength = max / (point_buf.size() / 100); // approx ?? points in every leaf?

        std::cout << "Max Vox side length: " << maxVoxSideLength << std::endl;
    }

    // update root parameters

    this->center = centroid;
    this->sideLength[0] = len_x;
    this->sideLength[1] = len_y;
    this->sideLength[2] = len_z;
    
    std::vector<Eigen::Vector3d*> tlf, trf, tlb, trb, blf, brf, blb, brb; // pointers for 8 subtrees
    
    begin = std::chrono::steady_clock::now();
    
    if(point_buf.size() > 0)
    {
        // create new octrees
        // determine points which need to be passed on to the children
        // determine center points, sidelengths etc.
        for(auto &p : point_buf)
        {   
            // bottom [b]
            if(p.y() < centroid.y())
            {
                if(p.x() < centroid.x()) // left [l]
                {
                    if(p.z() < centroid.z()) // back [b]
                    {
                        blb.push_back(&p);
                    }
                    else // front [f]
                    {
                        blf.push_back(&p);
                    }
                }
                else // right [r]
                {
                    if(p.z() < centroid.z()) // back[b]
                    {
                        brb.push_back(&p);
                    }
                    else //front[f]
                    {
                        brf.push_back(&p);
                    }
                }
            }
            else // top [t]
            {
                if(p.x() < centroid.x()) // left [l]
                {
                    if(p.z() < centroid.z()) // back [b]
                    {
                        tlb.push_back(&p);
                    }
                    else // front [f]
                    {
                        tlf.push_back(&p);
                    }
                }
                else // right [r]
                {
                    if(p.z() < centroid.z()) // back[b]
                    {
                        trb.push_back(&p);
                    }
                    else //front[f]
                    {  
                        trf.push_back(&p);
                    }
                }
            }
            
        }

        // std::cout << "Subtree point sizes: " << std::endl
        //           << blb.size() << std::endl
        //           << blf.size() << std::endl
        //           << brb.size() << std::endl
        //           << brf.size() << std::endl
        //           << tlb.size() << std::endl
        //           << tlf.size() << std::endl
        //           << trb.size() << std::endl
        //           << trf.size() << std::endl;

        //top left front to bottom right back

        double new_sidelength[3] = {len_x / 2, len_y / 2, len_z / 2};

        // calculate new centroids
        Eigen::Vector3d tlf_a = this->center + Eigen::Vector3d(-len_x / 2, len_y/2, len_z /2);
        Eigen::Vector3d trf_a = this->center + Eigen::Vector3d(len_x / 2, len_y/2, len_z /2);
        Eigen::Vector3d tlb_a = this->center + Eigen::Vector3d(-len_x / 2, len_y/2, -len_z /2);
        Eigen::Vector3d trb_a = this->center + Eigen::Vector3d(len_x / 2, len_y/2, -len_z /2);
        Eigen::Vector3d blf_a = this->center + Eigen::Vector3d(-len_x / 2, -len_y/2, len_z /2);
        Eigen::Vector3d brf_a = this->center + Eigen::Vector3d(len_x / 2, -len_y/2, len_z /2);
        Eigen::Vector3d blb_a = this->center + Eigen::Vector3d(-len_x / 2, -len_y/2, -len_z /2);
        Eigen::Vector3d brb_a = this->center + Eigen::Vector3d(len_x / 2, -len_y/2, -len_z /2);

        this->children[0] = new Octree(tlf, new_sidelength, tlf_a);
        this->children[1] = new Octree(trf, new_sidelength, trf_a);
        this->children[2] = new Octree(tlb, new_sidelength, tlb_a);
        this->children[3] = new Octree(trb, new_sidelength, trb_a);
        this->children[4] = new Octree(blf, new_sidelength, blf_a);
        this->children[5] = new Octree(brf, new_sidelength, brf_a);
        this->children[6] = new Octree(blb, new_sidelength, blb_a);
        this->children[7] = new Octree(brb, new_sidelength, brb_a);
    }

    end = std::chrono::steady_clock::now();

    std::cout << "Used time to create octree = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    std::cout << "Num points per leaf [avg]: " << Octree::num_leafpoints_acc / Octree::num_leafs << std::endl;
    std::cout << "Num leafs: " << Octree::num_leafs << std::endl;
    // iterate through the points once, capture bounding box, center, sidelengths etc.

}

Octree::~Octree()
{
    // empty
}

/**
 * @brief Reconstructs the surface of the pointcloud via marching cubes, utilizig the octree structure
 * 
 * @param vertices 
 * @param indices 
 * @param nVert 
 * @param nFaces 
 */
void Octree::reconstruct(float* vertices, float* indices, int& nVert, int& nFaces)
{
    //todo: implement this
    return;
}
