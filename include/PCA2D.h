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


/***
 *  \brief A self implemented PCA using only 2 dim. That ensure to always have a normal estimation on z=0 plan.
 */
class PCA2D {
public:

    /** \brief Empty constructor
     */
    PCA2D() : m_computed(false), m_indices(new std::vector<int>), m_cloud(nullptr){}

    /**
     * \brief Perform the PCA computation.
     */
    void initCompute();

    /**
     *  \brief Provide a input  cloud pointer to deal with.
     *  @param cloud Cloud to deal with.
     */
    void setInputCloud(const ConstPtrCloud& cloud);

    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
     * \param indices indices a pointer to the indices that represent the input data.
     */
    void setIndices(const pcl::PointIndices& indices);
    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
     * \param indices indices a pointer to the indices that represent the input data.
     */
    void setIndices(const pcl::PointIndicesPtr& indices);
    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
     * \param indices indices a pointer to the indices that represent the input data.
     */
    void setIndices(const pcl::PointIndicesConstPtr& indices);
    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
     * \param indices indices a pointer to the indices that represent the input data.
     */
    void setIndices(const boost::shared_ptr<std::vector<int>>& indices);
    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
     * \param indices indices a pointer to the indices that represent the input data.
     */
    void setIndices(const std::vector<int>& indices);


    /** \brief Return the mean computed, if PCA isn't computed, it will perform it.
     *
     * @return The mean vector from PCA algorithm.
     */
    Eigen::Vector2f getMean() {
        if(!m_computed) this->initCompute();
        return m_mean;
    }

    /** \brief Return the eigen Vector in Eigen Matrix , if PCA isn't computed, it will perform it.
     *
     * @return The EigenVectors matrix from PCA algorithm.
     */
    Eigen::Matrix2f getEigenVectors() {
        if(!m_computed) this->initCompute();
        return m_eigen_vectors;
    }

    /** \brief Return the eigen Vector in Eigen Matrix , if PCA isn't computed, it will perform it.
     *
     * @return The EigenVectors matrix from PCA algorithm.
     */
    Eigen::Vector2f getEigenValues() {
        if(!m_computed) this->initCompute();
        return m_eigen_values;
    }

    /**
     *  \brief Project point into PCA cartesian system.
     * @param input Point to project.
     * @param projection Point to fill with projection.
     */
    void project(const Point & input, Point& projection);

    /**
     *  \brief Project input cloud into PCA cartesian system.
     * @param input cloud with points to project.
     * @param projection Cloud to fill with projected points.
     */
    void project(const ConstPtrCloud& in_cloud, PtrCloud& out_cloud);


private:


    bool m_computed; ///< True if PCA have been performed, false otherwise.
    IndicesPtr m_indices; ///< Indice to use with input cloud.
    ConstPtrCloud m_cloud;///< InputCloud to deal with.

    Eigen::Vector2f m_mean; ///< Centroid of input cloud
    Eigen::Matrix2f m_eigen_vectors; ///< Eigen vector in matrix form.
    Eigen::Vector2f m_eigen_values;///< Eigen scalar in vector.

};


#endif //_PCA2D_H
