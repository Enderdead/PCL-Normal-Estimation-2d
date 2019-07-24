//
// Created by localuser on 22/07/19.
//


#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <boost/make_shared.hpp>

#ifndef NORMAL_ESTIMATION_2D_POINTXYNORMAL_H
#define NORMAL_ESTIMATION_2D_POINTXYNORMAL_H

typedef pcl::PointXYZ PointInT;
typedef pcl::Normal  PointOutT;

class Normal2DEstimation: public pcl::Feature<PointInT, PointOutT>
{
public:
    typedef boost::shared_ptr<Normal2DEstimation> Ptr;
    typedef boost::shared_ptr<const Normal2DEstimation> ConstPtr;
    using pcl::Feature<PointInT, PointOutT>::feature_name_;
    using pcl::Feature<PointInT, PointOutT>::getClassName;
    using pcl::Feature<PointInT, PointOutT>::indices_;
    using pcl::Feature<PointInT, PointOutT>::input_;
    using pcl::Feature<PointInT, PointOutT>::surface_;
    using pcl::Feature<PointInT, PointOutT>::k_;
    using pcl::Feature<PointInT, PointOutT>::search_radius_;
    using pcl::Feature<PointInT, PointOutT>::search_parameter_;

    typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
    typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;

    /** \brief Empty constructor. */
    Normal2DEstimation ()
            : vpx_ (0)
            , vpy_ (0)
            , vpz_ (0)
            , covariance_matrix_ ()
            , xyz_centroid_ ()
            , use_sensor_origin_ (true)
    {
        feature_name_ = "Normal2DEstimation";
    };

    /** \brief Empty destructor */
    virtual ~Normal2DEstimation () {}

    /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param cloud the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param plane_parameters the plane parameters as: a, b, d (ax + by  + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    inline bool
    computePointNormal2d (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                        Eigen::Vector3f &line_parameters, float &curvature) // TODO
    {
        if (indices.size () < 2)
        {
            line_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
            curvature = std::numeric_limits<float>::quiet_NaN ();
            return false;
        }


        pcl::PCA<pcl::PointXYZ> pca;
        pcl::CentroidPoint<pcl::PointXYZ> centroid;

        for(int index: indices){
            centroid.add(cloud.points[index]);
        }

        pcl::PointXYZ center;
        centroid.get(center);
        pca.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr(&cloud));
        pca.setIndices(pcl::IndicesConstPtr(&indices));

        auto result = pca.getEigenVectors().col(1);//TODO verfié ca

        line_parameters(0) = result(0);
        line_parameters(1) = result(1);
        line_parameters(2) = -result(0)*center.x - result(1)*center.y;
        curvature = 0.0;

        return true;
    }

    /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param cloud the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param nx the resultant X component of the plane normal
      * \param ny the resultant Y component of the plane normal
      * \param nz the resultant Z component of the plane normal
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    inline bool
    computePointNormal2d (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                        float &nx, float &ny, float &nz, float &curvature)
    {
        if (indices.size () <= 1)
        {
            nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
            return false;
        }

        // Get the plane normal and surface curvature
        pcl::PCA<pcl::PointXYZ> pca;

        pca.setInputCloud(cloud.makeShared());
        pca.setIndices(boost::make_shared<std::vector<int>>(indices));

        auto result = pca.getEigenVectors().col(1);//TODO verfié ca

        nx = result(0);
        ny = result(1);
        nz = 0.0;
        curvature = 0.0;

        return true;
    }

    /** \brief Provide a pointer to the input dataset
      * \param cloud the const boost shared pointer to a PointCloud message
      */
    virtual inline void
    setInputCloud (const PointCloudConstPtr &cloud)
    {
        input_ = cloud;
        if (use_sensor_origin_)
        {
            vpx_ = input_->sensor_origin_.coeff (0);
            vpy_ = input_->sensor_origin_.coeff (1);
        }
    }
    /** \brief Set the viewpoint.
      * \param vpx the X coordinate of the viewpoint
      * \param vpy the Y coordinate of the viewpoint
      * \param vpz the Z coordinate of the viewpoint
      */
    inline void
    setViewPoint (float vpx, float vpy)
    {
        vpx_ = vpx;
        vpy_ = vpy;
        use_sensor_origin_ = false;
    }

    /** \brief Get the viewpoint.
      * \param [out] vpx x-coordinate of the view point
      * \param [out] vpy y-coordinate of the view point
      * \param [out] vpz z-coordinate of the view point
      * \note this method returns the currently used viewpoint for normal flipping.
      * If the viewpoint is set manually using the setViewPoint method, this method will return the set view point coordinates.
      * If an input cloud is set, it will return the sensor origin otherwise it will return the origin (0, 0, 0)
      */
    inline void
    getViewPoint (float &vpx, float &vpy)
    {
        vpx = vpx_;
        vpy = vpy_;
    }

    /** \brief sets whether the sensor origin or a user given viewpoint should be used. After this method, the
      * normal estimation method uses the sensor origin of the input cloud.
      * to use a user defined view point, use the method setViewPoint
      */
    inline void
    useSensorOriginAsViewPoint ()
    {
        use_sensor_origin_ = true;
        if (input_)
        {
            vpx_ = input_->sensor_origin_.coeff (0);
            vpy_ = input_->sensor_origin_.coeff (1);
            vpz_ = input_->sensor_origin_.coeff (2);
        }
        else
        {
            vpx_ = 0;
            vpy_ = 0;
            vpz_ = 0;
        }
    }

    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
   * \param point a given point
   * \param vp_x the X coordinate of the viewpoint
   * \param vp_y the X coordinate of the viewpoint
   * \param vp_z the X coordinate of the viewpoint
   * \param normal the plane normal to be flipped
   * \ingroup features
   */
    inline void
    flipNormalTowardsViewpoint (const PointInT &point, float vp_x, float vp_y, float vp_z,
                                Eigen::Matrix<double, 3, 1>& normal)// TODO verif
    {
        Eigen::Matrix <double, 3, 1> vp (vp_x - point.x, vp_y - point.y, 0.);

        // Dot product between the (viewpoint - point) and the plane normal
        float cos_theta = vp.dot (normal);

        // Flip the plane normal
        if (cos_theta < 0)
        {
            normal *= -1;

        }
    }


    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
* \param point a given point
* \param vp_x the X coordinate of the viewpoint
* \param vp_y the X coordinate of the viewpoint
* \param vp_z the X coordinate of the viewpoint
* \param normal the plane normal to be flipped
* \ingroup features
*/
    inline void
    flipNormalTowardsViewpoint (const PointInT &point, float vp_x, float vp_y, float vp_z,
                                float& n_x,float& n_y,float& n_z)// TODO verif
    {
        Eigen::Matrix <double, 2, 1> vp (vp_x - point.x, vp_y - point.y);
        Eigen::Matrix <double, 2, 1> normal(n_x, n_y);
        // Dot product between the (viewpoint - point) and the plane normal
        float cos_theta = vp.dot (normal);

        // Flip the plane normal
        if (cos_theta < 0)
        {
            normal *= -1;

        }
        n_x = normal(0);
        n_y = normal(1);
        n_z = 0.0;

    }

protected:
    /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
      * setSearchSurface () and the spatial locator in setSearchMethod ()
      * \note In situations where not enough neighbors are found, the normal and curvature values are set to NaN.
      * \param output the resultant point cloud model dataset that contains surface normals and curvatures
      */
    void
    computeFeature (PointCloudOut &output) {
        // Allocate enough space to hold the results
        // \note This resize is irrelevant for a radiusSearch ().
        std::vector<int> nn_indices(k_);
        std::vector<float> nn_dists(k_);

        output.is_dense = true;
        // Save a few cycles by not checking every point for NaN/Inf values if the cloud is set to dense
        if (input_->is_dense) {
            // Iterating over the entire index vector
            for (size_t idx = 0; idx < indices_->size(); ++idx) {
                if (this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0) {
                    output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();

                    output.is_dense = false;
                    continue;
                }

                computePointNormal2d(*surface_, nn_indices,
                                   output.points[idx].normal[0], output.points[idx].normal[1],
                                   output.points[idx].normal[2], output.points[idx].curvature);

                flipNormalTowardsViewpoint(input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                           output.points[idx].normal[0], output.points[idx].normal[1],
                                           output.points[idx].normal[2]);

            }
        } else {
            // Iterating over the entire index vector
            for (size_t idx = 0; idx < indices_->size(); ++idx) {
                if (!isFinite((*input_)[(*indices_)[idx]]) ||
                    this->searchForNeighbors((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0) {
                    output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN();

                    output.is_dense = false;
                    continue;
                }

                computePointNormal2d(*surface_, nn_indices,
                                   output.points[idx].normal[0], output.points[idx].normal[1],
                                   output.points[idx].normal[2], output.points[idx].curvature);

                flipNormalTowardsViewpoint(input_->points[(*indices_)[idx]], vpx_, vpy_, vpz_,
                                           output.points[idx].normal[0], output.points[idx].normal[1],
                                           output.points[idx].normal[2]);

            }

        }
    }
    /** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
      * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
    float vpx_, vpy_, vpz_;


    /** \brief boolean to know if an intern copy have be done.
        */
    bool use_intern_copy_;

    /** \brief Placeholder for the 3x3 covariance matrix at each surface patch. */
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;

    /** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
    Eigen::Vector4f xyz_centroid_;

    /** whether the sensor origin of the input cloud or a user given viewpoint should be used.*/
    bool use_sensor_origin_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif //NORMAL_ESTIMATION_2D_POINTXYNORMAL_H
