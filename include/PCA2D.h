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

    PCA2D() : _computed(false), _indices(new std::vector<int>), _cloud(nullptr){}


    void initCompute();


    void setInputCloud(const ConstPtrCloud& cloud);

    void setIndices(const pcl::PointIndices& indices);
    void setIndices(const pcl::PointIndicesPtr& indices);
    void setIndices(const pcl::PointIndicesConstPtr& indices);
    void setIndices(const boost::shared_ptr<std::vector<int>>& indices);
    void setIndices(const std::vector<int>& indices);


    Eigen::Vector2f getMean() {
        if(!_computed) this->initCompute();
        return _mean;
    }

    Eigen::Matrix2f getEigenVectors() {
        if(!_computed) this->initCompute();
        return _eigen_vectors;
    }

    Eigen::Vector2f getEigenValues() {
        if(!_computed) this->initCompute();
        return _eigen_values;
    }

    void project(const Point & input, Point& projection);

    void project(const ConstPtrCloud& in_cloud, PtrCloud& out_cloud);


private:


    bool _computed;
    IndicesPtr _indices;
    ConstPtrCloud _cloud;

    Eigen::Vector2f _mean;
    Eigen::Matrix2f _eigen_vectors;
    Eigen::Vector2f _eigen_values;

};


#endif //_PCA2D_H
