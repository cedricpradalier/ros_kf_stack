#ifndef LOCAL_PLAN_H
#define LOCAL_PLAN_H

#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace radial_plan {

    class LocalPlan {
        public:
            typedef enum {LEFT, RIGHT} Side;

        public:
            LocalPlan(Side side, float d_desired, float d_safety, double forward_range, double backward_range, double spatial_resolution, unsigned int num_angles);

            ~LocalPlan() {}

            // Update the node costs based on the point cloud.
            void updateCellCosts(const pcl::PointCloud<pcl::PointXYZ> & pointCloud);

            std::list<cv::Point2f> getOptimalPath(float K_initial_angle, float K_length, 
                    float K_turn, float K_dist);

            void saveCellMaps();

        protected:
            typedef std::multimap<float, cv::Point3i> Heap;
        protected:
            Side side;
            float d_desired;
            float d_safety;
            double forward_range;
            double backward_range;
            double spatial_resolution;
            unsigned int num_angles, num_safe, num_desired;

            cv::Point2i world2map(const cv::Point2f & P) {
                return cv::Point2i(round((P.x + backward_range)/spatial_resolution),
                        round((P.y+forward_range)/spatial_resolution));
            }
            cv::Point2f map2world(const cv::Point2i & P) {
                return cv::Point2f(-backward_range + P.x * spatial_resolution,
                        -forward_range + P.y * spatial_resolution);
            }
            // P = {x, y, theta}
            cv::Point3f conf2world(const cv::Point3i & P) {
                return cv::Point3f(-backward_range + P.y * spatial_resolution,
                        -forward_range + P.z * spatial_resolution,
                        -M_PI + P.x * 2*M_PI / num_angles);
            }

            cv::Point3i world2conf(const cv::Point3f & P) {
                double theta = remainder(P.z,2*M_PI);
                return cv::Point3i(((int)round((theta + M_PI)*num_angles/(2*M_PI)))%num_angles,
                        round((P.x + backward_range)/spatial_resolution),
                        round((P.y+forward_range)/spatial_resolution));
            }

            bool isInMap(const cv::Point2i & P) {
                return (P.x>=0) && (P.y>=0) && (P.x<occupancy_map.cols) && (P.y<occupancy_map.rows);
            }
            bool isInMap(const cv::Point2f & P) {
                return (P.x>=-backward_range) && (P.y>=-forward_range) && (P.x<=forward_range) && (P.y<=forward_range);
            }

            bool isInConf(const cv::Point3i & P) {
                return (P.x>=0) && (P.y>=0) && (P.z>=0) && (P.x<(signed)num_angles) && (P.y<occupancy_map.cols) && (P.z<occupancy_map.rows);
            }
            bool isInConf(const cv::Point3f & P) {
                return (P.x>=-backward_range) && (P.y>=-forward_range) && (P.x<=forward_range) && (P.y<=forward_range);
            }

            cv::Mat1b occupancy_map;
            cv::Mat1b dsafe_pattern;
            cv::Mat1b ddes_pattern;
            cv::Mat1b safety_map;
            cv::Mat1b desired_map;
    };

};

#endif // LOCAL_PLAN_H
