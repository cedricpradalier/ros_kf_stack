

#include <radial_plan/LocalPlan.h>

using namespace radial_plan;

LocalPlan::LocalPlan(Side side, float d_desired, float d_safety, double forward_range, double backward_range, double spatial_resolution, unsigned int num_angles):
    side(side), d_desired(d_desired), d_safety(d_safety), forward_range(forward_range), backward_range(backward_range), spatial_resolution(spatial_resolution), num_angles(num_angles)
{
    occmap_size = cv::Size(round((backward_range + forward_range)/spatial_resolution),
            round(2*forward_range/spatial_resolution));
    occupancy_map = cv::Mat1b(occmap_size);
    safety_map = cv::Mat1b(occmap_size);
    int nsafe = ceil(2*d_safety/spatial_resolution);
    num_safe = nsafe | 1; // make it odd
    dsafe_pattern = cv::Mat1b(nsafe,nsafe);
    dsafe_pattern = 0xFF;
    int r = num_safe / 2;
    cv::circle(dsafe_pattern,cv::Point(r,r), r, 0, -1);

    safety_map = 0xFF;
    occupancy_map = 0x00;

    rotations.resize(num_angles);
    rotocc.resize(num_angles);
    distances.resize(num_angles);
    for (unsigned int i=0;i<num_angles;i++) {
        rotations[i] = cv::getRotationMatrix2D(
                cv::Point2f(backward_range/spatial_resolution,forward_range/spatial_resolution),
                i*360./num_angles, 1.0);
        distances[i] = cv::Mat1f(occmap_size,-1.0);
        rotocc[i] = cv::Mat1b(occmap_size);
    }
}


void LocalPlan::updateCellCosts(const pcl::PointCloud<pcl::PointXYZ> & pointCloud) {
    occupancy_map = 0x00;
    safety_map = 0xFF;
    ROS_INFO("%d points in point cloud",(int)pointCloud.size());
    for (size_t i=0;i<pointCloud.size();++i) {
        cv::Point2i P = world2map(cv::Point2f(pointCloud[i].x,pointCloud[i].y));
        if (isInMap(P)) {
            occupancy_map(P) = 0xFF;
        }
    }
    // Compute the safety map by pasting the safety pattern on the occupancy
    // grid, basically a big dilate from a morphology point of view
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
        }
    }

    // Now compute the distance from the safety map, but only orthogonally
    for (unsigned int i=0;i<num_angles;i++) {
        cv::warpAffine(safety_map,rotocc[i],rotations[i],occmap_size, cv::INTER_NEAREST, 
                cv::BORDER_CONSTANT, cv::Scalar(0xFF));
        for (int c=0;c<rotocc[i].cols;++c) {
            bool distance_to_something = false;
            if (rotocc[i](0,c)==0x00) {
                distance_to_something = true;
                distances[i](0,c) = 0.0;
            } else {
                distances[i](0,c) = -1.0;
            }

            for (int r=1;r<rotocc[i].rows;++r) {
                if (rotocc[i](r,c)==0x00) {
                    distance_to_something = true;
                    distances[i](r,c) = 0.0;
                } else if (!distance_to_something) {
                    distances[i](r,c) = -1.0;
                } else {
                    distances[i](r,c) = distances[i](r-1,c) + 1.0;
                }
            }
        }
    }
}

void LocalPlan::saveCellMaps() {
    cv::imwrite("occupancy_map.png",occupancy_map);
    cv::imwrite("safety_map.png",safety_map);
    cv::imwrite("dsafe_pattern.png",dsafe_pattern);
    for (unsigned int i=0;i<num_angles;i++) {
        char C[2] = {'A' + i, 0};
        cv::imwrite(std::string("rotocc")+std::string(C)+".png",rotocc[i]);
        cv::imwrite(std::string("rotation")+std::string(C)+".png",rotations[i]);
        cv::imwrite(std::string("distance")+std::string(C)+".png",distances[i]);
    }
}

std::list<cv::Point2f> LocalPlan::getOptimalPath(float K_initial_angle, float K_length, 
                    float K_turn, float K_dist) {

    assert(num_angles == 8); // other angles not yet implemented
    assert(K_length < K_turn * (2*M_PI)/sqrt(2.));
    cv::Point3i neighbours[24] = {
        // theta = 0 deg
        cv::Point3i(0,+1,0), // +1 on x
        cv::Point3i(+1,+1,+1), // move +1 on x, +1 on y, and rotate +45 deg
        cv::Point3i(-1,+1,-1), // move +1 on x, -1 on y, and rotate -45 deg
        // theta = 45 deg
        cv::Point3i(0,+1,+1), 
        cv::Point3i(+1,0,+1), 
        cv::Point3i(-1,+1,0), 
        // theta = 90 deg
        cv::Point3i(0,0,+1), 
        cv::Point3i(+1,-1,+1), 
        cv::Point3i(-1,+1,+1), 
        // theta = 135 deg
        cv::Point3i(0,-1,+1), 
        cv::Point3i(+1,-1,0), 
        cv::Point3i(-1,0,+1), 
        // theta = 180 deg
        cv::Point3i(0,-1,0), 
        cv::Point3i(+1,-1,-1), 
        cv::Point3i(-1,-1,+1), 
        // theta = 225 deg
        cv::Point3i(0,-1,-1), 
        cv::Point3i(+1,0,-1), 
        cv::Point3i(-1,-1,0), 
        // theta = 270 deg
        cv::Point3i(0,0,-1), 
        cv::Point3i(+1,+1,-1), 
        cv::Point3i(-1,-1,-1), 
        // theta = 315 deg
        cv::Point3i(0,+1,-1), 
        cv::Point3i(+1,+1,0), 
        cv::Point3i(-1,0,-1) 
    };
    float cost[6] = {
        // theta = 0 deg
        -K_length,
        -sqrt(2)*K_length + (M_PI/4) * K_turn,
        -sqrt(2)*K_length + (M_PI/4) * K_turn,
        // theta = 45 deg
        -K_length*sqrt(2),
        -K_length + (M_PI/4) * K_turn,
        -K_length + (M_PI/4) * K_turn,
    };
    cv::Point3i start = world2conf(cv::Point3f(0.0,0.0,0.0));
    cv::Mat1f cell_value[num_angles];
    cv::Mat3s predecessor[num_angles];
    for (unsigned int i=0;i<num_angles;i++) {
        cell_value[i] = cv::Mat1f(occmap_size);
        cell_value[i] = NAN;
        predecessor[i] = cv::Mat3s(occmap_size);
    }
    Heap heap;
    heap.insert(Heap::value_type(0.0, start));
    cell_value[start.x](start.y,start.z) = 0;
    size_t n_pop = 0;
    while (!heap.empty()) {
        n_pop += 1;
        // Select the cell at the top of the heap
        Heap::iterator hit = heap.begin();
        // the cell it contains is this_cell
        cv::Point3i this_cell = hit->second;
        // and its score is this_cost
        float this_cost = cell_value[this_cell.x](this_cell.y,this_cell.z);
        // We can remove it from the heap now.
        heap.erase(hit);
        // Now see where we can go from this_cell
        for (unsigned int i=0;i<3;i++) {
            cv::Point3i trans = neighbours[this_cell.x*3 + i];
            cv::Point3i dest = this_cell + trans;
            dest.x = (dest.x + num_angles) % num_angles;
            if (!isInConf(dest)) {
                // outside the grid
                continue;
            }
            uint8_t og = safety_map(dest.y,dest.z);
            if (og != 0xFF) {
                // occupied or unknown
                continue;
            }
            float d = distances[dest.x](dest.y,dest.z);
            float error = 0.0;
            if (d > 0) { // can't be equal because otherwise, we would be with the safety limits
                error = d*spatial_resolution - (d_desired - d_safety);
                error *= error;
            } else {
                // we are left or right of nothing
                error = 10.0;
            }
            float cv = cell_value[dest.x](dest.y,dest.z);
            float new_cost = this_cost + cost[(this_cell.z&1)*3 + i] + K_dist*error;
            if (isnan(cv) || (new_cost < cv)) {
                // found shortest path (or new path), updating the
                // predecessor and the value of the cell
                predecessor[dest.x](dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                cell_value[dest.x](dest.y,dest.z) = new_cost;
                // And insert the selected cells in the map.
                heap.insert(Heap::value_type(new_cost,dest));
            }
        }
    }
    ROS_INFO("N pops: %ld", n_pop);

    return std::list<cv::Point2f>();
}

