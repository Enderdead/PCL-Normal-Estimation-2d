//
// Created by Francois Gauthier-Clerc on 02/08/19.
//

#include "../include/Normal2dEstimation.h"


void Normal2dEstimation::setInputCloud(const ConstPtrCloud& cloud) {
    m_in_cloud = cloud;
    m_indices->clear();
    m_indices->resize (cloud->points.size ());
    for (unsigned int i = 0; i < cloud->points.size (); ++i) { (*m_indices)[i] = i; }
}


void Normal2dEstimation::setIndices(const pcl::PointIndices::Ptr& indices) {
    m_indices->clear();
    m_indices->resize(indices->indices.size());
    std::copy(indices->indices.cbegin(), indices->indices.cend(), m_indices->begin());
}

void Normal2dEstimation::setIndices(const pcl::PointIndices::ConstPtr& indices) {
    m_indices->clear();
    m_indices->resize(indices->indices.size());
    std::copy(indices->indices.cbegin(), indices->indices.cend(), m_indices->begin());
}


int Normal2dEstimation::searchForNeighbors(int index, std::vector<int>& nn_indices,std::vector<float>& nn_dists) const {
    nn_indices.clear();
    nn_dists.clear();
    if(m_k==0) {
        this->m_kd_tree->radiusSearch(index, m_search_radius, nn_indices, nn_dists, 0);
    }else {
        this->m_kd_tree->nearestKSearch(index, m_k, nn_indices, nn_dists);
    }
    return nn_indices.size();
}




void Normal2dEstimation::compute(const PtrCloud& normal_cloud) const {
// Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    boost::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(m_k));
    std::vector<float> nn_dists(m_k);

    normal_cloud->points.resize(m_in_cloud->points.size());
    normal_cloud->height = m_in_cloud->height;
    normal_cloud->width = m_in_cloud->width;

    if ((m_k==0) && (m_search_radius==0)) {
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch !");
    }
    if ((m_k!=0) && (m_search_radius!=0)){
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch (not both) !");
    }

    this->m_kd_tree->setInputCloud(m_in_cloud, m_indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (m_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if(this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) ==0 ) {
                normal_cloud->points[idx].x = normal_cloud->points[idx].y = normal_cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                       normal_cloud->points[idx].z);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                             normal_cloud->points[idx].z);

        }
    } else {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if (!isFinite(m_in_cloud->points[(*m_indices)[idx]]) ||
                this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].x = normal_cloud->points[idx].y = normal_cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                       normal_cloud->points[idx].z);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].x, normal_cloud->points[idx].y,
                                             normal_cloud->points[idx].z);
        }
    }
}



void Normal2dEstimation::compute(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud) const {

    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    boost::shared_ptr<std::vector<int>> nn_indices(new std::vector<int>(m_k));
    std::vector<float> nn_dists(m_k);

    normal_cloud->points.resize(m_in_cloud->points.size());
    normal_cloud->height = m_in_cloud->height;
    normal_cloud->width = m_in_cloud->width;

    if( (m_k==0) && (m_search_radius==0)){
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch !");
    }

    if( (m_k!=0) && (m_search_radius!=0)) {
        throw std::runtime_error("You must call once either setRadiusSearch or setKSearch (not both) !");
    }

    this->m_kd_tree->setInputCloud(m_in_cloud, m_indices);

    normal_cloud->is_dense = true;
    // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
    if (m_in_cloud->is_dense) {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if(this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) ==0 ) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();
                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
                                             normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                             normal_cloud->points[idx].normal_z);

        }
    } else {
        // Iterating over the entire index vector
        for (unsigned int idx = 0; idx < m_indices->size(); ++idx) {
            if (!isFinite(m_in_cloud->points[(*m_indices)[idx]]) ||
                this->searchForNeighbors((*m_indices)[idx], *nn_indices, nn_dists) == 0) {
                normal_cloud->points[idx].normal_x = normal_cloud->points[idx].normal_y = normal_cloud->points[idx].normal_z = std::numeric_limits<float>::quiet_NaN();

                normal_cloud->is_dense = false;
                continue;
            }

            this->computePointNormal2d(nn_indices,
                                       normal_cloud->points[idx].normal_x, normal_cloud->points[idx].normal_y,
                                       normal_cloud->points[idx].normal_z, normal_cloud->points[idx].curvature);

            this->flipNormalTowardsViewpoint(m_in_cloud->points[(*m_indices)[idx]],
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
        vect_x = m_in_cloud->points[(*indices)[0]].x - m_in_cloud->points[(*indices)[1]].x;
        vect_y = m_in_cloud->points[(*indices)[0]].y - m_in_cloud->points[(*indices)[1]].y;
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

    pca.setInputCloud(m_in_cloud);
    pca.setIndices(indices);
    auto result = pca.getEigenVectors().col(1);
    nx = result(0);
    ny = result(1);
    nz = 0.0;
    return true;

}

bool Normal2dEstimation::computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                           float &nx, float &ny, float &nz, float& curvature) const {
    if (indices->size () < 2)
    {
        nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
        return false;
    }
    if (indices->size()==2){
        double norm, vect_x, vect_y;
        vect_x = m_in_cloud->points[(*indices)[0]].x - m_in_cloud->points[(*indices)[1]].x;
        vect_y = m_in_cloud->points[(*indices)[0]].y - m_in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        nx = -vect_y;
        ny = vect_x;
        nz = 0.0;
        curvature = 0.0;
        return true;
    }


    // Get the plane normal and surface curvature
    PCA2D pca;

    pca.setInputCloud(m_in_cloud);
    pca.setIndices(indices);
    auto result = pca.getEigenVectors().col(1);
    nx = result(0);
    ny = result(1);
    nz = 0.0;
    curvature = pca.getEigenValues()(1) /  (pca.getEigenValues()(0)  + pca.getEigenValues()(1) );
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
        vect_x = m_in_cloud->points[(*indices)[0]].x - m_in_cloud->points[(*indices)[1]].x;
        vect_y = m_in_cloud->points[(*indices)[0]].y - m_in_cloud->points[(*indices)[1]].y;
        norm = std::pow(std::pow(vect_x,2.0)+std::pow(vect_y,2.0), 0.5);
        vect_x /= norm;
        vect_y /= norm;
        line_parameters(0) = -vect_y;
        line_parameters(1) = vect_x;
        line_parameters(2) = 0.0;
        return true;
    }

    pcl::PCA<Point> pca;

    pca.setInputCloud(m_in_cloud);
    pca.setIndices(indices);

    auto result = pca.getEigenVectors().col(1);

    line_parameters(0) = result(0);
    line_parameters(1) = result(1);
    line_parameters(2) = 0.0;
    return true;

}




void Normal2dEstimation::flipNormalTowardsViewpoint (const Point &point, float& x, float& y, float& z) const {
    Eigen::Matrix <double, 3, 1> normal (x, y, z);
    Eigen::Matrix <double, 3, 1> vp (m_view_point.x - point.x, m_view_point.y - point.y, 0.);

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
    Eigen::Matrix <double, 3, 1> vp (m_view_point.x - point.x, m_view_point.y - point.y, 0.);

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = vp.dot (normal);

    // Flip the plane normal
    if (cos_theta < 0)
    {
        normal *= -1;

    }
}