#include "octree.h"

double Octree::maxVoxSideLength;
std::vector<Eigen::Vector3d> Octree::point_buf;
int Octree::num_leafpoints_acc;
int Octree::num_leafs;
int Octree::kd;
DistanceFunction *Octree::dist_func;

// internal private constructor, used to build the octree
Octree::Octree(std::vector<Eigen::Vector3d*>& points, double new_sidelengths[3], Eigen::Vector3d& new_centroid, int layer)
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
        //std::cout << "[Octree] LEAF_LAYER: " << layer << std::endl;

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
    Eigen::Vector3d tlf_a = this->center + Eigen::Vector3d(-len_x / 4, len_y / 4, len_z / 4);
    Eigen::Vector3d trf_a = this->center + Eigen::Vector3d(len_x / 4, len_y / 4, len_z / 4);
    Eigen::Vector3d tlb_a = this->center + Eigen::Vector3d(-len_x / 4, len_y / 4, -len_z / 4);
    Eigen::Vector3d trb_a = this->center + Eigen::Vector3d(len_x / 4, len_y / 4, -len_z / 4);
    Eigen::Vector3d blf_a = this->center + Eigen::Vector3d(-len_x / 4, -len_y / 4, len_z / 4);
    Eigen::Vector3d brf_a = this->center + Eigen::Vector3d(len_x / 4, -len_y / 4, len_z / 4);
    Eigen::Vector3d blb_a = this->center + Eigen::Vector3d(-len_x / 4, -len_y / 4, -len_z / 4);
    Eigen::Vector3d brb_a = this->center + Eigen::Vector3d(len_x / 4, -len_y / 4, -len_z / 4);

    this->children[0] = new Octree(tlf, new_sidelength, tlf_a, layer + 1);
    this->children[1] = new Octree(trf, new_sidelength, trf_a, layer + 1);
    this->children[2] = new Octree(tlb, new_sidelength, tlb_a, layer + 1);
    this->children[3] = new Octree(trb, new_sidelength, trb_a, layer + 1);
    this->children[4] = new Octree(blf, new_sidelength, blf_a, layer + 1);
    this->children[5] = new Octree(brf, new_sidelength, brf_a, layer + 1);
    this->children[6] = new Octree(blb, new_sidelength, blb_a, layer + 1);
    this->children[7] = new Octree(brb, new_sidelength, brb_a, layer + 1);
}

/**
 * @brief Construct a new Octree:: Octree object
 *  public constructor used to build the tree, reads the transmitted filename!
 * 
 * @param filename   name of the file containing the point information
 */
Octree::Octree(std::string filename, int num_args, double maxSideLength, int ki, int kd, int kn)
{
    maxVoxSideLength = maxSideLength;

    Octree::kd = kd;

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
                std::cout << "finished reading the data" << std::endl;
                break;
            }
        }
        else if(num_args == 6)
        {
            if(!(file >> x >> y >> z >> r >> g >> b))
            {
                std::cout << "finished reading the data" << std::endl;
                break;
            }
        }
        else if(num_args == 3)
        {
            if(!(file >> x >> y >> z))
            {
                std::cout << "finished reading the data" << std::endl;
                break;
            }
        }
        else if(num_args == 5)
        {
            if(!(file >> x >> y >> z >> r >> g))
            {
                std::cout << "finished reading the data" << std::endl;
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

    dist_func = new DistanceFunction(point_buf, ki, kd, kn);
    double dist = dist_func->distance(-0.942292, 0.001739, -1.290601,10);

    std::cout << "Start iterating through the box to determine the needed parameters." << std::endl;
    begin = std::chrono::steady_clock::now();

    double max_x, max_y, max_z, min_x, min_y, min_z;
    max_x = max_y = max_z = std::numeric_limits<double>::min();
    min_x = min_y = min_z = std::numeric_limits<double>::max();

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

    }


    end = std::chrono::steady_clock::now();

 
  
    //std::cout << "Used time for O(n) iteration = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    double len_x = max_x - min_x;
    double len_y = max_y - min_y;
    double len_z = max_z - min_z;

    std::cout << "Initial Bounding Box side-lengths: " << len_x << " | " << len_y << " | " << len_z << std::endl;
    std::cout << "Initial Bounding Box MAX: " << max_x << " | " << max_y << " | " << max_z << std::endl;
    std::cout << "Initial Bounding Box MIN: " << min_x << " | " << min_y << " | " << min_z << std::endl;

    // error handling, determine max distance manually...
    if( maxVoxSideLength <= 0)
    {
        double max = std::max(len_x, len_y);
        max = std::max(max, len_z);

        std::cout << "Max side length: " << max << std::endl;
        maxVoxSideLength = max / (point_buf.size() / 100); // approx ?? points in every leaf?

        std::cout << "Max Vox side length: " << maxVoxSideLength << std::endl;
    }

    // update root parameters

    this->center = Eigen::Vector3d(min_x + (len_x / 2), min_y + (len_y / 2), min_z + (len_z / 2));
    this->sideLength[0] = len_x;
    this->sideLength[1] = len_y;
    this->sideLength[2] = len_z;

    std::cout << "Finished itering " << point_buf.size() << " points. Center is: " << std::endl << center << std::endl;
    
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
            if(p.y() < this->center.y())
            {
                if(p.x() < this->center.x()) // left [l]
                {
                    if(p.z() < this->center.z()) // back [b]
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
                    if(p.z() < this->center.z()) // back[b]
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
                if(p.x() < this->center.x()) // left [l]
                {
                    if(p.z() < this->center.z()) // back [b]
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
                    if(p.z() < this->center.z()) // back[b]
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
        Eigen::Vector3d tlf_a = this->center + Eigen::Vector3d(-len_x / 4, len_y / 4, len_z / 4);
        Eigen::Vector3d trf_a = this->center + Eigen::Vector3d(len_x / 4, len_y / 4, len_z / 4);
        Eigen::Vector3d tlb_a = this->center + Eigen::Vector3d(-len_x / 4, len_y / 4, -len_z / 4);
        Eigen::Vector3d trb_a = this->center + Eigen::Vector3d(len_x / 4, len_y / 4, -len_z / 4);
        Eigen::Vector3d blf_a = this->center + Eigen::Vector3d(-len_x / 4, -len_y / 4, len_z / 4);
        Eigen::Vector3d brf_a = this->center + Eigen::Vector3d(len_x / 4, -len_y / 4, len_z / 4);
        Eigen::Vector3d blb_a = this->center + Eigen::Vector3d(-len_x / 4, -len_y / 4, -len_z / 4);
        Eigen::Vector3d brb_a = this->center + Eigen::Vector3d(len_x / 4, -len_y / 4, -len_z / 4);  

        this->children[0] = new Octree(tlf, new_sidelength, tlf_a, 1);
        this->children[1] = new Octree(trf, new_sidelength, trf_a, 1);
        this->children[2] = new Octree(tlb, new_sidelength, tlb_a, 1);
        this->children[3] = new Octree(trb, new_sidelength, trb_a, 1);
        this->children[4] = new Octree(blf, new_sidelength, blf_a, 1);
        this->children[5] = new Octree(brf, new_sidelength, brf_a, 1);
        this->children[6] = new Octree(blb, new_sidelength, blb_a, 1);
        this->children[7] = new Octree(brb, new_sidelength, brb_a, 1);
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
void Octree::reconstruct(std::vector<double>& vertices, std::vector<int>& indices, int& nVert, int& nFaces)
{

    if(this->is_leaf)
    {

        nVert++;

        auto curr_proc = (nVert * 100) / Octree::num_leafs;

        //if((nVert / Octree::num_leafs) == (Octree::num_leafs / 100))
        if (curr_proc > nFaces)
        {
            nFaces ++;

            std::cout << nFaces << " Prozent" << std::endl; 
        }


        //std::cout << " we got a leaf" << std::endl;
        // calculate distance function for each corner point, determine if outside or inside iso surface
        int index = 0;

        double center_x = this->center[0];
        double center_y = this->center[1];
        double center_z = this->center[2];

        // calculate hoppes distance function for each of the corner vertices of the current leaf

        double side_x = this->sideLength[0];
        double side_y = this->sideLength[1];
        double side_z = this->sideLength[2];
        


        double dists[8];
        dists[0] = dist_func->distance(center_x - side_x / 2, center_y - side_y / 2, center_z + side_z / 2, Octree::kd);
        dists[1] = dist_func->distance(center_x + side_x / 2, center_y - side_y / 2, center_z + side_z / 2, Octree::kd);
        dists[2] = dist_func->distance(center_x + side_x / 2, center_y + side_y / 2, center_z + side_z / 2, Octree::kd);
        dists[3] = dist_func->distance(center_x - side_x / 2, center_y + side_y / 2, center_z + side_z / 2, Octree::kd);
        dists[4] = dist_func->distance(center_x - side_x / 2, center_y - side_y / 2, center_z - side_z / 2, Octree::kd);
        dists[5] = dist_func->distance(center_x + side_x / 2, center_y - side_y / 2, center_z - side_z / 2, Octree::kd);
        dists[6] = dist_func->distance(center_x + side_x / 2, center_y + side_y / 2, center_z - side_z / 2, Octree::kd);
        dists[7] = dist_func->distance(center_x - side_x / 2, center_y + side_y / 2, center_z - side_z / 2, Octree::kd);

        
        int cubeIndex = 0;
        for(int i = 0; i < 8; i++)
        {
            if(dists[i] > 0)
            {
                cubeIndex |= 1 << i;
            }
        }


        double side_xh = side_x / 2;
        double side_yh = side_y / 2;
        double side_zh = side_z / 2;

        std::vector<Eigen::Vector3d> edgeLookup = {
            this->center + Eigen::Vector3d(0,        -side_yh, side_zh),
            this->center + Eigen::Vector3d(side_xh,  0,        side_zh),
            this->center + Eigen::Vector3d(0,        side_yh,  side_zh),
            this->center + Eigen::Vector3d(-side_xh, 0,        side_zh),
            this->center + Eigen::Vector3d(0,        -side_yh, -side_zh),
            this->center + Eigen::Vector3d(side_xh,  0,        -side_zh),
            this->center + Eigen::Vector3d(0,        side_yh,  -side_zh),
            this->center + Eigen::Vector3d(-side_xh, 0,        -side_zh),
            this->center + Eigen::Vector3d(-side_xh, -side_yh, 0),
            this->center + Eigen::Vector3d(side_xh,  -side_yh, 0),
            this->center + Eigen::Vector3d(side_xh,  side_yh, 0),
            this->center + Eigen::Vector3d(-side_xh, side_yh, 0)
        };

        


        //std::cout << "CubeIndex " << cubeIndex << std::endl;

        //std::cout << "finished calculating the distances" << std::endl;

        //std::cout << "Dists: " << dist_1 << " | " << dist_2 << " | " << dist_3 << " | " << dist_4 << " | " << dist_5 << " | " << dist_6 << " | " << dist_7 << " | " << dist_8 << " | " << std::endl;
        
        int i = 0;
        while(lvr2::MCTable[cubeIndex][i] != -1)
        {
            int cur_index = vertices.size() / 3;

            for(int j = 0; j < 3; j++)
            {
                int edge_no = lvr2::MCTable[cubeIndex][i+j];
                auto vertex = edgeLookup[edge_no];
                vertices.push_back(vertex[0]);
                vertices.push_back(vertex[1]);
                vertices.push_back(vertex[2]);

                indices.push_back(cur_index);
                cur_index += 1;
            }
            i += 3;
        }

        return;
    }

    for(int i = 0; i < 8; i++)
    {
        this->children[i]->reconstruct(vertices, indices, nVert, nFaces);
    }

    // finished iteration


    return;
}
