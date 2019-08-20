//
// Created by Francois Gauthier-Clerc on 05/08/19.
//
//
// Created by Francois Gauthier-Clerc on 02/08/19.
//

#ifndef _NORMAL2DESTIMATION_H
#define _NORMAL2DESTIMATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/PointIndices.h>
#include <vector>
#include <pcl/common/pca.h>
#include <boost/make_shared.hpp>
#include "./PCA2D.h"


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point>::Ptr PtrCloud;
typedef pcl::PointCloud<Point>::ConstPtr ConstPtrCloud;
typedef pcl::search::Search<Point>::Ptr PtrkdTree;
typedef boost::shared_ptr<std::vector<int> > IndicesPtr;


class Normal2dEstimation {
public:

    Normal2dEstimation(double search_radius=0.0, unsigned int k=0) : m_search_radius(search_radius), m_k(k), m_indices(new std::vector<int>), m_kd_tree(nullptr), m_in_cloud(nullptr), m_view_point(0,0,0) {}


    void compute(const PtrCloud& normal_cloud) const;

    void compute(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud) const;

    void setViewPoint(const Point& origin)  {m_view_point = origin;}

    Point getViewPoint() const {return m_view_point;}

    void setRadiusSearch(double search_radius) {m_search_radius = search_radius;}

    void setSearchMethod(const PtrkdTree& kdtree) {m_kd_tree = kdtree;}

    void setKSearch(unsigned int k) {m_k =k;}

    int getKSearch() const {return m_k;}

    void setInputCloud(const ConstPtrCloud& cloud);

    ConstPtrCloud getInputCloud() const {return m_in_cloud;}

    void setIndices(const pcl::PointIndices::Ptr& indices);

    void setIndices(const pcl::PointIndices::ConstPtr& indices);


private:

    int searchForNeighbors(int index, std::vector<int>& nn_indices,std::vector<float>& nn_dists) const;


    bool computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                               float &nx, float &ny, float &nz) const;

    bool computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                               float &nx, float &ny, float &nz, float& curvature) const;

    bool computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                               Eigen::Vector3f &line_parameters) const;

    // Remet le normal dans le bon sens
    void flipNormalTowardsViewpoint (const Point &point, Eigen::Matrix<double, 3, 1>& normal) const;


    void flipNormalTowardsViewpoint (const Point &point, float& x, float& y, float& z) const;



    double m_search_radius;
    unsigned int m_k;

    IndicesPtr m_indices;
    PtrkdTree m_kd_tree;
    ConstPtrCloud m_in_cloud;


    Point m_view_point;
};


#endif //_NORMAL2DESTIMATION_H
