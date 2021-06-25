#include "distance_function.h"


DistanceFunction::DistanceFunction(std::vector<Eigen::Vector3d> &points, int ki, int kd, int kn) : kd(kd), kn(kn), ki(ki)
{
    // first: calculate normals

    
    // fill flann dataset 
    std::cout << points.size() << std::endl;
    
    dataset_ = flann::Matrix<double>(new double[points.size() * 3], points.size(), 3);

    std::cout << "[DistanceFunction] Initialized target data array" << std::endl;
    for(int i = 0; i < points.size(); i++)
    {
        dataset_[i][0] = (points[i].x());
        dataset_[i][1] = (points[i].y());
        dataset_[i][2] = (points[i].z());
    }

    std::cout << "[DistanceFunction] Initialized flann dataset" << std::endl;
    index = new flann::Index<flann::L2<double>>(dataset_, flann::KDTreeSingleIndexParams(10, false));

    std::cout << "Initialized index..." << std::endl;

    // std::cout << __LINE__ << std::endl;

    index->buildIndex();

    std::cout << "[DistanceFunction] Build flann index!" << std::endl;

    // now estimate normals based on the flann kd tree


}

DistanceFunction::~DistanceFunction()
{
    delete[] dataset_.ptr();
}

DistanceFunction::DistanceFunction()
{
    // emtpy default constructor
}


double DistanceFunction::distance(double x, double y, double z, int k)
{
    // find nearest neighbor (k nearst), go through each of them, calculate distance via (-1|+1) * (p-c)*n

    double querydata[3];

    flann::Matrix<double> query_point(new double[3], 1, 3);

    query_point[0][0] = x;
    query_point[0][1] = y;
    query_point[0][2] = z;


    std::vector<std::vector<int>> indices, next_indices;
    std::vector<std::vector<double>> dists;


    std::chrono::steady_clock::time_point begin, end;

    begin = std::chrono::steady_clock::now();

    // calculate the KNNs
    DistanceFunction::index->knnSearch(query_point, indices, dists, k, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

    std::cout << "vector sizes " << indices.size() << " | " << indices[0].size() << std::endl;

    end = std::chrono::steady_clock::now();

    std::cout << "[DistanceFunction] Time used for knn search = " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << "[ns]" << std::endl;
    
    // calculate normals on demand


    std::vector<Eigen::Vector3d> normals;

    Eigen::Vector3d centroid;
    Eigen::Matrix3d covariance_matrix;

    std::cout << "[DistanceFunction] calculating normals for the on demand neighbors" << std::endl;

    for(int i = 0; i < indices[0].size(); i++)
    {
        std::cout << __LINE__ << std::endl;

        query_point[0][0] = dataset_[indices[0][i]][0];
        query_point[0][1] = dataset_[indices[0][i]][1];
        query_point[0][2] = dataset_[indices[0][i]][2];

        std::cout << __LINE__ << std::endl;

        centroid = Eigen::Vector3d::Zero();
        covariance_matrix = Eigen::Matrix3d::Zero();

        // // calculate the KNNs
        DistanceFunction::index->knnSearch(query_point, next_indices, dists, this->kn, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

        std::cout << __LINE__ << std::endl;



        //calculate centroid
        for(int j = 0; j < next_indices[0].size(); j++)
        {
            centroid += Eigen::Vector3d(dataset_[next_indices[0][j]][0], dataset_[next_indices[0][j]][1], dataset_[next_indices[0][j]][2]);            
        }

        std::cout << __LINE__ << std::endl;


        centroid /= kn;

        for(int j = 0; j < next_indices[0].size(); j++)
        {
            // calc current point moved by the clouds centroid
            Eigen::Vector3d cur_point(dataset_[next_indices[0][j]][0], dataset_[next_indices[0][j]][1], dataset_[next_indices[0][j]][2]);            
            Eigen::Vector3d coVec = cur_point - centroid; // sub neighbors centroid
            covariance_matrix += coVec * coVec.transpose();

        }

        std::cout << __LINE__ << std::endl;


        covariance_matrix /=  next_indices[0].size(); // fully calculated
        
        //std::cout << covariance_matrix << std::endl;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance_matrix);


        std::cout << __LINE__ << std::endl;

        if (eigensolver.info() != Eigen::Success) 
        {
            continue;
        }
        
        auto eigen_values = eigensolver.eigenvalues();
        //std::cout << "Eigenvalues: " << std::endl << eigen_values << std::endl;
        auto eigen_vectors = eigensolver.eigenvectors();
        //std::cout << "Eigenvectors: " << std::endl << eigen_vectors << std::endl;

        std::cout << __LINE__ << std::endl;


        int max_index;
        eigen_values.maxCoeff(&max_index);
        
        //std::cout << "Max coeff:" << max_index << std::endl;

        auto max_eigen_vec = eigen_vectors.col(0);
        
        // std::cout << "Max eigenvec: " << std::endl << max_eigen_vec << std::endl;

        std::cout << __LINE__ << std::endl;

        // add normal to normal array
        Eigen::Vector3d normal_direction(x, y, z);
        Eigen::Vector3d normal(max_eigen_vec.x(), max_eigen_vec.y(), max_eigen_vec.z());

        std::cout << __LINE__ << std::endl;

        if(normal.dot(normal_direction - Eigen::Vector3d(dataset_[indices[0][i]][0], dataset_[indices[0][i]][1], dataset_[indices[0][i]][2])) < 0)
        {

            normal *= -1;
        }

        normals.push_back(normal);
    }

    std::cout << "[DistanceFunction] finished calculating the normals" << std::endl;

    // calculate the distance function from hoppe for each of those..

    // for each nearest neighbor: average distances
    std::cout << __LINE__ << std::endl;

    double avg_dist = 0;

    for(int i = 0; i < indices[0].size(); i++)
    {
        Eigen::Vector3d cur_vec(dataset_[indices[0][i]][0], dataset_[indices[0][i]][1], dataset_[indices[0][i]][2]);
        std::cout << "cur vec: " << cur_vec << std::endl;
        std::cout << "cur normal:" << normals[i] << std::endl;

        double distance = (Eigen::Vector3d(x,y,z) - cur_vec).dot(normals[i]);

        std::cout << "[DistanceFunction] Calculated distance: " << distance << std::endl;

        avg_dist += distance;
    }

    std::cout << __LINE__ << std::endl;


    avg_dist /= indices[0].size();

    delete[] query_point.ptr();


    return avg_dist;
}
