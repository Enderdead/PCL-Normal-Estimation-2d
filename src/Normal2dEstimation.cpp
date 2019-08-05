//
// Created by Francois Gauthier-Clerc on 02/08/19.
//

#include "../include/Normal2dEstimation.h"


void Normal2dEstimation::setInputCloud(const ConstPtrCloud& cloud) {
    _in_cloud = cloud;
    _indices->clear();
    _indices->resize (cloud->points.size ());
    for (int i = 0; i < cloud->points.size (); ++i) { (*_indices)[i] = i; }
}


void Normal2dEstimation::setIndices(const pcl::PointIndices::Ptr& indices) {
    _indices->clear();
    _indices->resize(indices->indices.size());
    std::copy(indices->indices.cbegin(), indices->indices.cend(), _indices->begin());
}

void Normal2dEstimation::setIndices(const pcl::PointIndices::ConstPtr& indices) {
    _indices->clear();
    _indices->resize(indices->indices.size());
    std::copy(indices->indices.cbegin(), indices->indices.cend(), _indices->begin());
}


int Normal2dEstimation::searchForNeighbors(int index, std::vector<int>& nn_indices,std::vector<float>& nn_dists) const {
    nn_indices.clear();
    nn_dists.clear();
    if(_k==0) {
        this->_kd_tree->radiusSearch(index, _search_radius, nn_indices, nn_dists, 0);
    }else {
        this->_kd_tree->nearestKSearch(index, _k, nn_indices, nn_dists);
    }
    return nn_indices.size();
}




void Normal2dEstimation::compute(const PtrCloud& normal_cloud) const {
// Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    boost::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(_k));
    std::vector<float> nn_dists(_k);

    normal_cloud->points.resize(_in_cloud->points.size());
    normal_cloud->height = _in_cloud->height;
    normal_cloud->width = _in_cloud->width;

    assert(("You must call once either setRadiusSearch or setKSearch !", ((_k==0) && _search_radius==0)));

    assert(("You must call once either setRadiusSearch or setKSearch (not both) !", ((_k!=0) && _search_radius!=0)));


    this->_kd_tree->setInputCloud(_in_cloud, _indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (int idx = 0; idx < _indices->size(); ++idx) {
            if(this->searchForNeighbors((*_indices)[idx], *nn_indices, nn_dists) ==0 ) {
                normal_cloud->points[idx].x = normal_cloud->points[idx].y = normal_cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                       normal_cloud->points[idx].z);

            this->flipNormalTowardsViewpoint(_in_cloud->points[(*_indices)[idx]],
                                             normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                             normal_cloud->points[idx].z);

        }
    } else {
        // Iterating over the entire index vector
        for (int idx = 0; idx < _indices->size(); ++idx) {
            if (!isFinite(_in_cloud->points[(*_indices)[idx]]) ||
                this->searchForNeighbors((*_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].x = normal_cloud->points[idx].y = normal_cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                       normal_cloud->points[idx].z);

            this->flipNormalTowardsViewpoint(_in_cloud->points[(*_indices)[idx]],
                                             normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                             normal_cloud->points[idx].z);
        }
    }
}



void Normal2dEstimation::compute(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud) const {

    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    boost::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(_k));
    std::vector<float> nn_dists(_k);

    normal_cloud->points.resize(_in_cloud->points.size());
    normal_cloud->height = _in_cloud->height;
    normal_cloud->width = _in_cloud->width;

    assert(("You must call once either setRadiusSearch or setKSearch !", ((_k!=0) || _search_radius!=0)));

    assert(("You must call once either setRadiusSearch or setKSearch (not both) !", (   (_k!=0) != _search_radius!=0)));


    this->_kd_tree->setInputCloud(_in_cloud, _indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (int idx = 0; idx < _indices->size(); ++idx) {
            if(this->searchForNeighbors((*_indices)[idx], *nn_indices, nn_dists) ==0 ) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z);

            this->flipNormalTowardsViewpoint(_in_cloud->points[(*_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z);

        }
    } else {
        // Iterating over the entire index vector
        for (int idx = 0; idx < _indices->size(); ++idx) {
            if (!isFinite(_in_cloud->points[(*_indices)[idx]]) ||
                this->searchForNeighbors((*_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z);

            this->flipNormalTowardsViewpoint(_in_cloud->points[(*_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z);
        }
    }

}



bool Normal2dEstimation::computePointNormal2d (boost::shared_ptr<std::vector<int>> &indices,
                                               float &nx, float &ny, float &nz) const {

    if (indices->size () < 2)
    {
        nx = ny = nz = std::numeric_limits<float>::quiet_NaN ();
        return false;
    }
    if (indices->size()==2){
        double norm, vect_x, vect_y;
        vect_x = _in_cloud->points[(*indices)[0]].x - _in_cloud->points[(*indices)[1]].x;
        vect_y = _in_cloud->points[(*indices)[0]].y - _in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        nx = -vect_y;
        ny = vect_x;
        nz = 0.0;
        return true;
    }


    // Get the plane normal and surface curvature
    PCA2D pca;

    pca.setInputCloud(_in_cloud);
    pca.setIndices(indices);
    auto result = pca.getEigenVectors().col(1);
    auto result2 = pca.getEigenVectors();
    nx = result(0);
    ny = result(1);
    nz = 0.0;
    return true;

}

bool Normal2dEstimation::computePointNormal2d (boost::shared_ptr<std::vector<int>> &indices,
                                               Eigen::Vector3f &line_parameters) const {

    if (indices->size () < 2)
    {
        line_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
        return false;
    }


    if (indices->size()==2){
        double norm, vect_x, vect_y;
        vect_x = _in_cloud->points[(*indices)[0]].x - _in_cloud->points[(*indices)[1]].x;
        vect_y = _in_cloud->points[(*indices)[0]].y - _in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        line_parameters(0) = -vect_y;
        line_parameters(1) = vect_x;
        line_parameters(2) = 0.0;
        return true;
    }

    pcl::PCA<Point> pca;

    pca.setInputCloud(_in_cloud);
    pca.setIndices(indices);

    auto result = pca.getEigenVectors().col(1);

    line_parameters(0) = result(0);
    line_parameters(1) = result(1);
    line_parameters(2) = 0.0;
    return true;

}




void Normal2dEstimation::flipNormalTowardsViewpoint (const Point &point, float& x, float& y, float& z) const {
    Eigen::Matrix <double, 3, 1> normal (x, y, z);
    Eigen::Matrix <double, 3, 1> vp (_view_point.x - point.x, _view_point.y - point.y, 0.);

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = vp.dot (normal);
    // Flip the plane normal
    if (cos_theta < 0)
    {
        x *= -1;
        y *= -1;
        z *= -1;
    }
}



void Normal2dEstimation::flipNormalTowardsViewpoint (const Point &point, Eigen::Matrix<double, 3, 1>& normal) const {
    Eigen::Matrix <double, 3, 1> vp (_view_point.x - point.x, _view_point.y - point.y, 0.);

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = vp.dot (normal);

    // Flip the plane normal
    if (cos_theta < 0)
    {
        normal *= -1;

    }
}