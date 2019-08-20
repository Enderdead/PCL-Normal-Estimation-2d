//
// Created by Francois Gauthier-Clerc on 02/08/19.
//

#ifndef _PCA2D_H
#define _PCA2D_H

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/common/centroid.h>
#include <pcl/PointIndices.h>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point>::Ptr PtrCloud;
typedef pcl::PointCloud<Point>::ConstPtr ConstPtrCloud;
typedef pcl::search::Search<Point>::Ptr PtrkdTree;
typedef boost::shared_ptr<std::vector<int> > IndicesPtr;


class PCA2D {
public:

    PCA2D() : m_computed(false), m_indices(new std::vector<int>), m_cloud(nullptr){}


    void initCompute();


    void setInputCloud(const ConstPtrCloud& cloud);

    void setIndices(const pcl::PointIndices& indices);
    void setIndices(const pcl::PointIndicesPtr& indices);
    void setIndices(const pcl::PointIndicesConstPtr& indices);
    void setIndices(const boost::shared_ptr<std::vector<int>>& indices);
    void setIndices(const std::vector<int>& indices);


    Eigen::Vector2f getMean() {
        if(!m_computed) this->initCompute();
        return m_mean;
    }

    Eigen::Matrix2f getEigenVectors() {
        if(!m_computed) this->initCompute();
        return m_eigen_vectors;
    }

    Eigen::Vector2f getEigenValues() {
        if(!m_computed) this->initCompute();
        return m_eigen_values;
    }

    void project(const Point & input, Point& projection);

    void project(const ConstPtrCloud& in_cloud, PtrCloud& out_cloud);


private:


    bool m_computed;
    IndicesPtr m_indices;
    ConstPtrCloud m_cloud;

    Eigen::Vector2f m_mean;
    Eigen::Matrix2f m_eigen_vectors;
    Eigen::Vector2f m_eigen_values;

};


#endif //_PCA2D_H
