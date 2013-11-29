

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

    int ndes = ceil(2*d_desired/spatial_resolution);
    num_desired = ndes | 1; // make it odd
    // Now compute a half-circle of distances
    ddes_pattern[0] = cv::Mat1s(2*num_desired,2*num_desired,2*num_desired);
    ddes_pattern[1] = cv::Mat1s(2*num_desired,2*num_desired,2*num_desired);
    for (unsigned int r=0;r<2*num_desired;++r) {
        for (unsigned int c=0;c<2*num_desired;++c) {
            if (fabs(atan2(float(c)-num_desired,float(r))) < M_PI/3) {
                ddes_pattern[0](r,c) = (unsigned int)hypot(float(r),float(c)-num_desired);
                ddes_pattern[1](r,c) = (unsigned int)hypot(2*num_desired - float(r),float(c)-num_desired);
            }
        }
    }

    safety_map = 0xFF;
    occupancy_map = 0x00;

    desired_map.resize(num_angles);
    rotations.resize(num_angles);
    rotations_inv.resize(num_angles);
    rotocc.resize(num_angles);
    distances.resize(num_angles);
    for (unsigned int i=0;i<num_angles;i++) {
        rotations[i] = cv::getRotationMatrix2D(
                cv::Point2f(backward_range/spatial_resolution,forward_range/spatial_resolution),
                180. + i*360./num_angles, 1.0);
        cv::invertAffineTransform(rotations[i],rotations_inv[i]);
        distances[i] = cv::Mat1f(occmap_size,-1.0);
        rotocc[i] = cv::Mat1b(occmap_size);
        desired_map[i] = cv::Mat1s(occmap_size);
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
    for (int x=0;x < occupancy_map.cols; ++x) {
        for (int y=0;y < occupancy_map.rows; ++y) {
            if (!occupancy_map(y,x)) continue;

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
        cv::Mat1s dmap(occmap_size,num_desired*2);
        cv::warpAffine(occupancy_map,rotocc[i],rotations_inv[i],occmap_size, cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, 
                cv::BORDER_CONSTANT, cv::Scalar(0));
        for (int y=0;y < rotocc[i].rows; ++y) {
            for (int x=0;x < rotocc[i].cols; ++x) {
                if (!rotocc[i](y,x)) continue;

                int x_min_dest = std::max<int>(x - num_desired,0);
                int x_max_dest = std::min<int>(x + num_desired,occupancy_map.cols);
                int x_min_src = num_desired - x + x_min_dest;
                int x_max_src = num_desired + x_max_dest - x;
                int y_min_src, y_max_src, y_min_dest, y_max_dest;
                if (side == LEFT) {
                    y_min_dest = y;
                    y_max_dest = std::min<int>(y + 2*num_desired,occupancy_map.rows);
                    y_min_src = 0;
                    y_max_src = y_max_dest - y_min_dest;
                } else {
                    y_min_dest = std::max<int>(0,y - 2*num_desired);
                    y_max_dest = y;
                    y_min_src = 2*num_desired - (y_max_dest-y_min_dest);
                    y_max_src = 2*num_desired;
                }
                if ((y_max_src - y_min_src) == 0) continue; // nothing to do
                cv::Range s_src[2] = { cv::Range(y_min_src,y_max_src), cv::Range(x_min_src,x_max_src) };
                cv::Range s_dest[2] = { cv::Range(y_min_dest,y_max_dest), cv::Range(x_min_dest,x_max_dest) };
                dmap(s_dest) = cv::min(dmap(s_dest),ddes_pattern[side](s_src));
            }
        }
        dmap = cv::abs(dmap - d_desired/spatial_resolution);
        cv::warpAffine(dmap,desired_map[i],rotations[i],occmap_size, cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, 
                cv::BORDER_CONSTANT, cv::Scalar(2*d_desired));
#if 0
        cv::Mat1f D = cv::Mat1f(occmap_size,2*d_desired);
        unsigned int j = (side==RIGHT)?(num_angles/2):0;
        cv::warpAffine(safety_map,rotocc[i],rotations_inv[(i+j)%num_angles],occmap_size, cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, 
                cv::BORDER_CONSTANT, cv::Scalar(0xFF));
        for (int c=0;c<rotocc[i].cols;++c) {
            bool distance_to_something = false;
            if (rotocc[i](0,c)==0x00) {
                distance_to_something = true;
                D(0,c) = 0.0;
            } else {
                D(0,c) = 2*d_desired;
            }

            for (int r=1;r<rotocc[i].rows;++r) {
                if (rotocc[i](r,c)==0x00) {
                    distance_to_something = true;
                    D(r,c) = 0.0;
                } else if (!distance_to_something) {
                    D(r,c) = 2*d_desired;
                } else {
                    D(r,c) = D(r-1,c) + 1.0;
                }
            }
        }
        cv::warpAffine(D,distances[i],rotations[(i+j)%num_angles],occmap_size, cv::INTER_NEAREST | cv::WARP_INVERSE_MAP, 
                cv::BORDER_CONSTANT, cv::Scalar(2*d_desired));
#endif
#if 0
        char C[2] = {'A' + i, 0};
        cv::imwrite(std::string("drot")+std::string(C)+".png",D*5);
        cv::Mat1f E1 = cv::abs(distances[i] - (d_desired - d_safety)/spatial_resolution);
        cv::imwrite(std::string("error")+std::string(C)+".png",E1);
        FILE * fp = fopen((std::string("error")+std::string(C)+".png").c_str(),"w");
        for (int c=0;c<E1.cols;++c) {
            for (int r=1;r<E1.rows;++r) {
                fprintf(fp,"%.2f ", E1(r,c));
            }
            fprintf(fp,"\n");
        }
        fclose(fp);
#endif
    }
}

void LocalPlan::saveCellMaps() {
    cv::imwrite("occupancy_map.png",occupancy_map);
    cv::imwrite("safety_map.png",safety_map);
    cv::imwrite("dsafe_pattern.png",dsafe_pattern);
    cv::imwrite("ddes_pattern0.png",ddes_pattern[0]);
    cv::imwrite("ddes_pattern1.png",ddes_pattern[1]);
    for (unsigned int i=0;i<num_angles;i++) {
        char C[2] = {'A' + i, 0};
        cv::imwrite(std::string("rotocc")+std::string(C)+".png",rotocc[i]);
        //cv::imwrite(std::string("rotation")+std::string(C)+".png",rotations[i]);
        //cv::imwrite(std::string("distance")+std::string(C)+".png",distances[i]*5);
        cv::imwrite(std::string("desired")+std::string(C)+".png",desired_map[i]);
    }
}

std::list<cv::Point3f> LocalPlan::getOptimalPath(float K_initial_angle, float K_length, 
                    float K_turn, float K_dist) {

    assert(num_angles == 8); // other angles not yet implemented
    assert(K_length < K_turn * (2*M_PI)/sqrt(2.));
    cv::Point3i neighbours[24] = {
        // theta = 180 deg
        cv::Point3i(0,0,-1), 
        cv::Point3i(+1,-1,-1), 
        cv::Point3i(-1,+1,-1), 
        // theta = 225 deg
        cv::Point3i(0,-1,-1), 
        cv::Point3i(+1,-1,0), 
        cv::Point3i(-1,0,-1), 
        // theta = 270 deg
        cv::Point3i(0,-1,0), 
        cv::Point3i(+1,-1,+1), 
        cv::Point3i(-1,-1,-1), 
        // theta = 315 deg
        cv::Point3i(0,-1,+1), 
        cv::Point3i(+1,0,+1), 
        cv::Point3i(-1,-1,0) ,
        // theta = 0 deg
        cv::Point3i(0,0,+1), // +1 on x
        cv::Point3i(+1,+1,+1), // move +1 on x, +1 on y, and rotate +45 deg
        cv::Point3i(-1,-1,+1), // move +1 on x, -1 on y, and rotate -45 deg
        // theta = 45 deg
        cv::Point3i(0,+1,+1), 
        cv::Point3i(+1,+1,0), 
        cv::Point3i(-1,0,+1), 
        // theta = 90 deg
        cv::Point3i(0,+1,0), 
        cv::Point3i(+1,+1,-1), 
        cv::Point3i(-1,+1,+1), 
        // theta = 135 deg
        cv::Point3i(0,+1,-1), 
        cv::Point3i(+1,0,-1), 
        cv::Point3i(-1,+1,0), 
    };
    float cost[6] = {
        // theta = 0 deg
        -spatial_resolution*K_length,
        -spatial_resolution*sqrt(2)*K_length + (M_PI/4) * K_turn,
        -spatial_resolution*sqrt(2)*K_length + (M_PI/4) * K_turn,
        // theta = 45 deg
        -spatial_resolution*K_length*sqrt(2),
        -spatial_resolution*K_length + (M_PI/4) * K_turn,
        -spatial_resolution*K_length + (M_PI/4) * K_turn,
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
            cv::Point3f pw = conf2world(dest);
#if 0
            float d = distances[dest.x](dest.y,dest.z);
            float error = 0.0;
            if (d > 0) { // can't be equal because otherwise, we would be with the safety limits
                error = d*spatial_resolution - (d_desired - d_safety);
                error *= error;
            } else {
                // we are left or right of nothing
                error = 10.0;
            }
#else
            float error = desired_map[dest.x](dest.y,dest.z);
#endif
            float cv = cell_value[dest.x](dest.y,dest.z);
            float new_cost = this_cost + cost[(this_cell.x&1)*3 + i] + K_dist*error + K_initial_angle*fabs(pw.z); 
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
#if 0
    saveCellMaps();
    for (unsigned int i=0;i<num_angles;i++) {
        char C[2] = {'A' + i, 0};
        cv::imwrite(std::string("cv")+std::string(C)+".png",cell_value[i]);
    }
#endif
    double lowest_cv = NAN;
    cv::Point3i targeti;
    for (unsigned int i=0;i<num_angles;i++) {
        int r,c;
        c = 0;
        for (r=0;r<cell_value[i].rows;++r) {
            if (safety_map(r,c) != 0xFF) { continue; }
            if (isnan(cell_value[i](r,c))) { continue; }
            if (isnan(lowest_cv) || (cell_value[i](r,c) < lowest_cv)) {
                lowest_cv = cell_value[i](r,c);
                targeti = cv::Point3i(i,r,c);
            }
        }
        c = cell_value[i].cols - 1;
        for (r=0;r<cell_value[i].rows;++r) {
            if (safety_map(r,c) != 0xFF) { continue; }
            if (isnan(cell_value[i](r,c))) { continue; }
            if (isnan(lowest_cv) || (cell_value[i](r,c) < lowest_cv)) {
                lowest_cv = cell_value[i](r,c);
                targeti = cv::Point3i(i,r,c);
            }
        }
        r = 0;
        for (c=0;c<cell_value[i].cols;++c) {
            if (safety_map(r,c) != 0xFF) { continue; }
            if (isnan(cell_value[i](r,c))) { continue; }
            if (isnan(lowest_cv) || (cell_value[i](r,c) < lowest_cv)) {
                lowest_cv = cell_value[i](r,c);
                targeti = cv::Point3i(i,r,c);
            }
        }
        r = cell_value[i].rows - 1;
        for (c=0;c<cell_value[i].cols;++c) {
            if (safety_map(r,c) != 0xFF) { continue; }
            if (isnan(cell_value[i](r,c))) { continue; }
            if (isnan(lowest_cv) || (cell_value[i](r,c) < lowest_cv)) {
                lowest_cv = cell_value[i](r,c);
                targeti = cv::Point3i(i,r,c);
            }
        }
    }
    // ROS_INFO("Target value at %d %d %d: value %f",targeti.x,targeti.y,targeti.z,lowest_cv);
    if (isnan(lowest_cv)) {
        ROS_ERROR("No feasible destination to plan to");
        return std::list<cv::Point3f>();
    }
    std::list<cv::Point3f> lpath;
    lpath.push_front(conf2world(targeti));
    while (targeti != start) {
        const cv::Vec3s & p = predecessor[targeti.x](targeti.y,targeti.z);
        targeti = cv::Point3i(p[0],p[1],p[2]);
        lpath.push_front(conf2world(targeti));
        assert(lpath.size()<10000);
    }
    return lpath;
}

