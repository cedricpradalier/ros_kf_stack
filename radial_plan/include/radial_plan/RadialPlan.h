#ifndef RADIAL_PLAN_H
#define RADIAL_PLAN_H

#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <nabo/nabo.h>

#include "Eigen/Core"
#if EIGEN_VERSION_AT_LEAST(2,92,0)
	#define EIGEN3_API
#endif
#ifndef EIGEN3_API
	#include "Eigen/Array"
#endif

#define USE_FILTERED_OUTPUT

namespace radial_plan {

    // Description of a radial network where each node is defined by coordinates
    // (r,j,k) (integers), where
    // r is the distance to the origin
    // j is the angular index such that, the node cartesian coordinates are
    // (r*cos(j*pi/n), r*sin(j*pi/n)), with j in [-n/2, n/2] (n should be odd, to
    // make sure j can be zero and the range symmetric). 
    // k is the current heading offset with respect to j. 
    // Transitions are expected as follows:
    // (r,j,k) -> (r,j,k +/- 1) : pure rotation
    // (r,j,j) -> (r+1,j+k,k): spiral motion
    class RadialPlan {
        public:
            RadialPlan(unsigned int max_r, unsigned int num_angles, unsigned int num_connections,
                    bool filter_glare, float dist_scale = 1.0, float alpha_scale = M_PI);

            ~RadialPlan() {}

            typedef enum {LEFT, RIGHT} Side;

            // Update the node costs based on the point cloud.
            void updateNodeCosts(const pcl::PointCloud<pcl::PointXYZ> & pointCloud, Side side, float d_desired, float d_safety);

            std::list<cv::Point3f> getOptimalPath(float K_initial_angle, float K_length, 
                    float K_turn, float K_dist);

            boost::shared_ptr<Nabo::NNSearchF const> getNearestNeighbourSearch() {
                return nns;
            }

        protected:
            typedef std::multimap<float, cv::Point3i> Heap;
        protected:
            unsigned int n_r;
            unsigned int n_j;
            unsigned int n_k;
            float r_glare;
            bool filter_glare;
            float r_scale;
            float angle_scale;
            int conn_range, ang_range;

            struct NaboFilter : public Nabo::PointFilter {
                const Eigen::MatrixXf & nns_cloud;
                const Eigen::MatrixXf & nns_query;
                float x_ref, y_ref;
                float orientationOffset;
                float u_sep, v_sep;
                NaboFilter(const Eigen::MatrixXf & cloud, const Eigen::MatrixXf & query) : nns_cloud(cloud), nns_query(query){}
                virtual ~NaboFilter(){}

                virtual void setQueryIndex(int i_query) {
                    x_ref = nns_query(0,i_query); y_ref = nns_query(1,i_query); 
                    float alpha=atan2(y_ref,x_ref);
                    u_sep = cos(alpha + orientationOffset); v_sep = sin(alpha + orientationOffset);
                }
                void setOrientationOffset(float theta) {
                    orientationOffset = theta;
                }
                virtual bool acceptPoint(int index) const {
                    return (u_sep*(nns_cloud(0,index)-x_ref)+v_sep*(nns_cloud(1,index)-y_ref)) >= 0.0;
                }
            };

            boost::shared_ptr<Nabo::NNSearchF> nns;

            bool isInGrid(const cv::Point3i & P) {
                if ((P.x < 0) || (P.x >= (signed)n_r)) return false;
                if ((P.y < 0) || (P.y >= (signed)n_j)) return false;
                if ((P.z < 0) || (P.z >= (signed)n_k)) return false;
                return true;
            }

            // length of segment (r,0) -> (r+1, k), as a function of (r,k)
            cv::Mat_<float> transition_cost;
            // generic transition model; (r,j,k) -> (r+P.x, j+P.y, k+P.z)
            std::vector<cv::Point3i> neighbours;

            // Cost of each node, to be updated based on laser readings
            // as a function of (r,j,k)
            cv::Mat_<float> node_cost, node_safety;

            // Search matrix for nearest neighbours.
            Eigen::MatrixXf nns_cloud, nns_query;
            // The output of the query are a matrix of indices to columns of M and 
            // a matrix of squared distances corresponding to these indices.
            // These matrix must have the correct size when passed to the search function.
            Eigen::MatrixXi indices;
            Eigen::MatrixXf dists2;
    };

};

#endif // RADIAL_PLAN_H
