

#include <radial_plan/LocalPlan.h>

using namespace radial_plan;

LocalPlan::LocalPlan(Side side, float d_desired, float d_safety, double forward_range, double backward_range, double spatial_resolution, unsigned int num_angles):
    side(side), d_desired(d_desired), d_safety(d_safety), forward_range(forward_range), backward_range(backward_range), spatial_resolution(spatial_resolution), num_angles(num_angles)
{
    occupancy_map = cv::Mat1b(round(2*forward_range/spatial_resolution),
            round((backward_range + forward_range)/spatial_resolution));
    safety_map = cv::Mat1b(occupancy_map.rows,occupancy_map.cols);
    int nsafe = ceil(2*d_safety/spatial_resolution);
    num_safe = nsafe | 1; // make it odd
    dsafe_pattern = cv::Mat1b(nsafe,nsafe);
    dsafe_pattern = 0xFF;
    int r = num_safe / 2;
    cv::circle(dsafe_pattern,cv::Point(r,r), r, 0, -1);

    int dim[3] = {num_angles,occupancy_map.rows,occupancy_map.cols};
    desired_map = cv::Mat1b(3,dim);
    int ndes = ceil(2*d_desired/spatial_resolution);
    num_desired = ndes | 1; // make it odd
    dim[1] = num_desired; dim[2] = num_desired;
    ddes_pattern = cv::Mat1b(3,dim);
    ddes_pattern = 0xFF;

    desired_map = 0xFF;
    safety_map = 0xFF;
    occupancy_map = 0x00;

    for (unsigned int i=0;i<num_angles;i++) {
        cv::Mat1b slice = cv::Mat1b(num_desired,num_desired,ddes_pattern.data+i*ddes_pattern.step[0]);
        int w = num_desired / 2;
        double alpha = remainder((i-0.5)*360./num_angles + ((side==RIGHT)?90.:-90.), 360.); 
        // TODO: add side dependency
        cv::ellipse(slice,cv::Point(w,w), cv::Size(w,w), 0.0, alpha, alpha + 360./num_angles, 0, 1);
    }
}


void LocalPlan::updateCellCosts(const pcl::PointCloud<pcl::PointXYZ> & pointCloud) {
    occupancy_map = 0x00;
    safety_map = 0xFF;
    desired_map = 0xFF;
    ROS_INFO("%d points in point cloud",(int)pointCloud.size());
    for (size_t i=0;i<pointCloud.size();++i) {
        cv::Point2i P = world2map(cv::Point2f(pointCloud[i].x,pointCloud[i].y));
        if (isInMap(P)) {
            occupancy_map(P) = 0xFF;
        }
    }
    size_t nocc = 0;
    for (int x=0;x < occupancy_map.cols; ++x) {
        for (int y=0;y < occupancy_map.rows; ++y) {
            if (!occupancy_map(y,x)) continue;
            nocc += 1;

            int x_min_dest = std::max<int>(x - num_safe/2,0);
            int x_max_dest = std::min<int>(x + num_safe/2,occupancy_map.cols);
            int y_min_dest = std::max<int>(y - num_safe/2,0);
            int y_max_dest = std::min<int>(y + num_safe/2,occupancy_map.rows);
            int x_min_src = num_safe/2 - x + x_min_dest;
            int x_max_src = num_safe/2 + x_max_dest - x;
            int y_min_src = num_safe/2 - y + y_min_dest;
            int y_max_src = num_safe/2 + y_max_dest - y;
            cv::Range s_src[2] = { cv::Range(y_min_src,y_max_src), cv::Range(x_min_src,x_max_src) };
            cv::Range s_dest[2] = { cv::Range(y_min_dest,y_max_dest), cv::Range(x_min_dest,x_max_dest) };
            safety_map(s_dest) &= dsafe_pattern(s_src);

            x_min_dest = std::max<int>(x - num_desired/2,0);
            x_max_dest = std::min<int>(x + num_desired/2,occupancy_map.cols);
            y_min_dest = std::max<int>(y - num_desired/2,0);
            y_max_dest = std::min<int>(y + num_desired/2,occupancy_map.rows);
            x_min_src = num_desired/2 - x + x_min_dest;
            x_max_src = num_desired/2 + x_max_dest - x;
            y_min_src = num_desired/2 - y + y_min_dest;
            y_max_src = num_desired/2 + y_max_dest - y;
            cv::Range d_src[3] = { cv::Range::all(), cv::Range(y_min_src,y_max_src), cv::Range(x_min_src,x_max_src) }; 
            cv::Range d_dest[3] = { cv::Range::all(), cv::Range(y_min_dest,y_max_dest), cv::Range(x_min_dest,x_max_dest) }; 
            desired_map(d_dest) &= ddes_pattern(d_src);
        }
    }
    ROS_INFO("%d cell occupied",(int)nocc);
}

void LocalPlan::saveCellMaps() {
    cv::imwrite("occupancy_map.png",occupancy_map);
    cv::imwrite("safety_map.png",safety_map);
    cv::imwrite("dsafe_pattern.png",dsafe_pattern);
    for (unsigned int i=0;i<num_angles;i++) {
        cv::Mat1b slice = cv::Mat1b(num_desired,num_desired,ddes_pattern.data+i*ddes_pattern.step[0]);
        char C[2] = {'A' + i, 0};
        cv::imwrite(std::string("ddes_pattern")+std::string(C)+".png",slice);
        slice = cv::Mat1b(occupancy_map.rows,occupancy_map.cols,desired_map.data+i*desired_map.step[0]);
        cv::imwrite(std::string("desired_map")+std::string(C)+".png",slice);
    }
}

std::list<cv::Point2f> LocalPlan::getOptimalPath(float K_initial_angle, float K_length, 
                    float K_turn, float K_dist) {
    return std::list<cv::Point2f>();
}

