//
// Created by Francois Gauthier-Clerc on 02/08/19.
//

#include "../include/PCA2D.h"


void PCA2D::initCompute() {
    assert(("You have to set a cloud before ask any result !", _cloud ));


    pcl::CentroidPoint<Point> centroid;
    // Compute mean
    _mean = Eigen::Vector2f::Zero ();
    Eigen::Vector4f temp_mean;
    compute3DCentroid (*_cloud, *_indices, temp_mean);
    _mean(0) = temp_mean(0);
    _mean(1) = temp_mean(1);


    // Compute demeanished cloud
    Eigen::MatrixXf cloud_demean;
    demeanPointCloud (*_cloud, *_indices, temp_mean, cloud_demean);
    assert (cloud_demean.cols () == int (_indices->size ()));
    // Compute the product cloud_demean * cloud_demean^T
    Eigen::Matrix2f alpha = static_cast<Eigen::Matrix2f> (cloud_demean.topRows<2> () * cloud_demean.topRows<2> ().transpose ());

    // Compute eigen vectors and values
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> evd (alpha);
    // Organize eigenvectors and eigenvalues in ascendent order
    for (int i = 0; i < 2; ++i)
    {
        _eigen_values[i] = evd.eigenvalues ()[1-i];
        _eigen_vectors.col (i) = evd.eigenvectors ().col (1-i);
    }
    // If not basis only then compute the coefficients

    _computed = true;

}


void PCA2D::setInputCloud(const ConstPtrCloud& cloud) {
    _computed = false;
    _cloud = cloud;

}

void PCA2D::setIndices(const pcl::PointIndices& indices) {
    _computed = false;
    _indices->clear();
    _indices->insert( _indices->begin(), indices.indices.cbegin(), indices.indices.cend());
}
void PCA2D::setIndices(const pcl::PointIndicesPtr& indices) {
    _computed = false;
    _indices->insert( _indices->begin(), indices->indices.cbegin(), indices->indices.cend());

}
void PCA2D::setIndices(const pcl::PointIndicesConstPtr& indices) {
    _computed = false;
    _indices->insert( _indices->begin(), indices->indices.cbegin(), indices->indices.cend());

}


void PCA2D::setIndices(const boost::shared_ptr<std::vector<int>>& indices) {
    _computed = false;
    _indices->insert( _indices->begin(), indices->cbegin(), indices->cend());
}