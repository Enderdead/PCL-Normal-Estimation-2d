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



/** \brief Normal2dEstimation estimate local straight line at each points using it neighbour. Contrary to native normal
  * estimation  on PCL lib, it works only on 2D points cloud.
  *
  * \note This class use 3D points cloud but ignore all z components. So make sure you have all your points on z=0 plan.
  *
  *
  */
class Normal2dEstimation {
public:

    /**
     * \brief Classic constructor, you can directly setup parameters using constructor arguments.
     *
     * @param search_radius If different from zero, the normal estimation will use the radius to find out which points
     *                      it take for line estimation.
     *
     * @param k If different from zero, the normal estimation will find the kth closest points for computing line estimation.
     */
    Normal2dEstimation(double search_radius=0.0, unsigned int k=0) : m_search_radius(search_radius), m_k(k), m_indices(new std::vector<int>), m_kd_tree(nullptr), m_in_cloud(nullptr), m_view_point(0,0,0) {}


    /**
     * \brief Compute on each point the normal estimation.
     * @param normal_cloud Normal cloud to fill with normal estimation result.
     */
    void compute(const PtrCloud& normal_cloud) const;


    /**
     * \brief Compute on each point the normal estimation.
     * @param normal_cloud Normal cloud to fill with normal estimation result.
     */
    void compute(const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud) const;

    /**
     *  \brief Setup the view point
     *
     *  The view points is used in normal orientation purpose. This orientation force all normals to look at view points.
     *
     * @param origin View point position.
     */
    void setViewPoint(const Point& origin)  {m_view_point = origin;}

    /**
     * \brief return the current view point.
     */
    Point getViewPoint() const {return m_view_point;}

    /**
     *  \brief Setup a search radius for normal estimation on each point.
     * @param search_radius Max distance for search radius (cartesian distance).
     */
    void setRadiusSearch(double search_radius) {m_search_radius = search_radius;}

    /**
     * \brief Prove current radius for normal estimation.
     * @return Current search radius.
     */
    inline double getRadiusSearch() const {return m_search_radius;}

    /** \brief Provide a pointer to the search object.
     * \param kdtree tree a pointer to the spatial search object.
     */
    void setSearchMethod(const PtrkdTree& kdtree) {m_kd_tree = kdtree;}


    /**
     * \brief Setup the cluster size for normal estimation.
     * @param k cluster size for normal estimation.
     */
    void setKSearch(unsigned int k) {m_k =k;}

    /**
     * \brief Provide the current k constant.
     * @return k constant.
     */
    inline int getKSearch() const {return m_k;}

    /**
     *  \brief Provide a input  cloud pointer to deal with.
     *  @param cloud Cloud to deal with.
     */
    void setInputCloud(const ConstPtrCloud& cloud);

    /**
     * \brief Provide the current cloud set as input.
     * @return A pointer to current input cloud.
     */
    inline ConstPtrCloud getInputCloud() const {return m_in_cloud;}

    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
      * \param indices indices a pointer to the indices that represent the input data.
      */
    void setIndices(const pcl::PointIndices::Ptr& indices);

    /**
     * \brief Provide a pointer to the vector of indices that represents the input data.
      * \param indices indices a pointer to the indices that represent the input data.
      */
    void setIndices(const pcl::PointIndices::ConstPtr& indices);


private:

    /** \brief Search for nearest neighbors using the spatial locator from search class and class parameters (k and search_radius).
      *
      * \param index index the index of the query point.
      * \param[out] nn_indices the resultant vector of indices representing the neighbors cluster.
      * \param[out] nn_dists the resultant vector of distances representing the distances from the query point to the neighbors.
      *
      * \return The number of neighbors found. If no neighbors are found or an error occurred, return 0.
      */
    int searchForNeighbors(int index, std::vector<int>& nn_indices,std::vector<float>& nn_dists) const;


    /** \brief Compute the Least-Squares straight line fit for a given set of points, using their indices,
      * and return the estimated normal parameters.

      * \param indices the point cloud indices that need to be used.
        * \param nx the resultant X component of the plane normal.
        * \param ny the resultant Y component of the plane normal.
        * \param nz the resultant Z component of the plane normal (always 0).
      * @return Return true if a normal can be computed, false 	otherwise.
      */
    bool computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                               float &nx, float &ny, float &nz) const;

    /** \brief Compute the Least-Squares straight line fit for a given set of points, using their indices,
      * and return the estimated normal parameters.
      *
      * \param indices the point cloud indices that need to be used.
      * \param nx the resultant X component of the plane normal.
      * \param ny the resultant Y component of the plane normal.
      * \param nz the resultant Z component of the plane normal (always 0).
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1)
      * \f]
      *
      * @return Return true if a normal can be computed, false otherwise.
      */
    bool computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                               float &nx, float &ny, float &nz, float& curvature) const;
    /** \brief Compute the Least-Squares straight line fit for a given set of points, using their indices,
      * and return the estimated normal parameters.
      * \param indices the point cloud indices that need to be used.
      * \param normal parameter.
      *
      * @return Return true if a normal can be computed, false otherwise.
      */
    bool computePointNormal2d (boost::shared_ptr<std::vector<int>>& indices,
                               Eigen::Vector3f &line_parameters) const;


    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint.
     *
     * @param point The normal location.
     * @param normal The normal vector to transform.
     */
   void flipNormalTowardsViewpoint (const Point &point, Eigen::Matrix<double, 3, 1>& normal) const;

    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint.
     *
     * @param point The normal location.
     * @param x normal x component.
     * @param y normal y component.
     * @param z normal z component.
     */
   void flipNormalTowardsViewpoint (const Point &point, float& x, float& y, float& z) const;



   double m_search_radius; ///< Max distance between the estimate normal position with each neighbour for take them on normal estimation.
   unsigned int m_k; ///< Max cluster size for estimate normal.

   IndicesPtr m_indices;///< Indices from input cloud.
   PtrkdTree m_kd_tree; ///< Search object to figure out neighbours.
   ConstPtrCloud m_in_cloud; ///< Input cloud pointer to deal with.


   Point m_view_point; ///< View points used for normal fliping (default (0,0)).
};


#endif //_NORMAL2DESTIMATION_H
