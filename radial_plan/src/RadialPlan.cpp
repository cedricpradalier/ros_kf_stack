#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <ros/ros.h>
#include <nabo/nabo.h>
#include "radial_plan/RadialPlan.h"

using namespace radial_plan;
using namespace Nabo;
using namespace Eigen;

// #define SAVE_OUTPUT
// #define SIGNED_DISTANCE

RadialPlan::RadialPlan(unsigned int max_r, unsigned int num_angles, unsigned int num_connections,
        float dist_scale, float alpha_scale) :
    n_r(max_r+1), n_j(num_angles), n_k(num_connections), r_scale(dist_scale), angle_scale(alpha_scale)
{
    // n_k and n_j must be odd
    assert(n_k & 1);
    assert(n_j & 1);
    conn_range = n_k/2;
    ang_range = n_j/2;
    neighbours.push_back(cv::Point3i(0,0,+1));
    neighbours.push_back(cv::Point3i(0,0,-1));
    neighbours.push_back(cv::Point3i(1,0,0));
    // for (int k=-conn_range;k<=conn_range;k++) {
    //     neighbours.push_back(cv::Point3i(1,k,0));
    // }
#ifdef SAVE_OUTPUT
    printf("Transitions:\n");
    for (unsigned int i=0;i<neighbours.size();i++) {
        printf("%d: %d %d %d\n",i,neighbours[i].x,neighbours[i].y,neighbours[i].z);
    }
#endif
#ifdef SAVE_OUTPUT
    printf("Transition cost:\n");
#endif
    transition_cost = cv::Mat_<float>(n_r,n_k);
    for (int r=0;r<(signed)(n_r-1);r++) {
        float x0 = r, y0 = 0.0; 
        for (int k=-conn_range;k<=conn_range; k++) {
            float x1 = (r+1) * r_scale * cos (k*angle_scale/(2*ang_range));
            float y1 = (r+1) * r_scale * sin (k*angle_scale/(2*ang_range));
            float d = hypot(y1-y0,x1-x0);
            transition_cost(r,conn_range+k) = d*d;
#ifdef SAVE_OUTPUT
            printf("%d,%d: (%.2f,%.2f) -> (%.2f, %.2f): %.2f\n",r,k,x0,y0,x1,y1,d*d);
#endif
        }
    }
#ifdef SIGNED_DISTANCE
    int dims[3] = {n_r,n_j,n_k};
    node_cost = cv::Mat_<float>(3,dims);
#else
    node_cost = cv::Mat_<float>(n_r,n_j);
#endif
    nns_query = MatrixXf(2, n_r*n_j) ;
#ifdef SAVE_OUTPUT
    printf("Node list:\n");
#endif
    for (unsigned int r = 0; r < n_r; r ++ ) {
        for (int j=-ang_range;j <= ang_range; j++) {
            unsigned int i = r*n_j + j + ang_range;
            nns_query(0,i) = r * r_scale * cos(j*angle_scale/(2*ang_range));
            nns_query(1,i) = r * r_scale * sin(j*angle_scale/(2*ang_range));
#ifdef SAVE_OUTPUT
            printf("Node (%d,%d): %6.2f %6.2f\n",r,j,nns_query(0,i),nns_query(1,i));
#endif
        }
    }
    indices.resize(1, nns_query.cols());
    dists2.resize(1, nns_query.cols());
}

// TODO: add a way to specify the cost as a designed signed distance. 
void RadialPlan::updateNodeCosts(const pcl::PointCloud<pcl::PointXYZ> & pointCloud, float d_desired, float d_safety) {
#ifdef SAVE_OUTPUT
    FILE *pc = fopen("pc","w");
#endif
    std::vector<float> r_max(n_j,NAN);
    nns.reset();
   	nns_cloud.resize(2, pointCloud.size());
    for (unsigned int i=0;i<pointCloud.size();i++) {
        nns_cloud(0,i) = pointCloud[i].x;
        nns_cloud(1,i) = pointCloud[i].y;
        float alpha = round(atan2(pointCloud[i].y, pointCloud[i].x) * 2 * ang_range / angle_scale);
        int i_alpha = (int)alpha;
        if (abs(i_alpha) <= ang_range) {
            float r = hypot(pointCloud[i].y,pointCloud[i].x);
            if (isnan(r_max[ang_range + i_alpha]) || (r < r_max[ang_range + i_alpha])) {
                r_max[ang_range + i_alpha] = r;
            }
        }
#ifdef SAVE_OUTPUT
        fprintf(pc,"%e %e %e\n",pointCloud[i].x,pointCloud[i].y,0.0);
#endif
    }
#ifdef SAVE_OUTPUT
    fclose(pc);
#endif

#ifdef SAVE_OUTPUT
    FILE * bd = fopen("bd", "w");
    float dalpha = angle_scale / (4*ang_range);
    for (int j=0;j<(signed)n_j;j++) {
        float alpha = (j - ang_range)*angle_scale / (2*ang_range);
        fprintf(bd,"%e %e\n%e %e\n%e %e\n",
                r_max[j]*cos(alpha-dalpha), r_max[j]*sin(alpha-dalpha), 
                r_max[j]*cos(alpha), r_max[j]*sin(alpha), 
                r_max[j]*cos(alpha+dalpha), r_max[j]*sin(alpha+dalpha));
    }
    fclose(bd);
#endif


    // Create a kd-tree for M, note that M must stay valid during the lifetime of the kd-tree.
    nns.reset(NNSearchF::createKDTreeLinearHeap(nns_cloud));


    // Look for the nearest neighbours of each query point, 
    // We do not want approximations but we want to sort by the distance,
    nns->knn(nns_query, indices, dists2, 1, 0, 0);

#ifdef SAVE_OUTPUT
    FILE * fp = fopen("nodes","w");
    printf("Node costs\n");
#endif
    for (int j=0;j < (signed)n_j; j++) {
        float alpha = (j-ang_range) * angle_scale / (2*ang_range);
        for (unsigned int r = 0; r < n_r; r ++ ) {
            unsigned int ix = r*n_j + j;
            float d = sqrt(dists2(0,ix));
#if SIGNED_DISTANCE
            float dx_near = nns_cloud(0,indices(0,ix)) - r*r_scale*cos(alpha);
            float dy_near = nns_cloud(1,indices(0,ix)) - r*r_scale*sin(alpha);
            for (unsigned int k=0;k<n_k;k++) {
                // Compute the side of the closest point, to make sure the
                // distance is signed 
                float beta = (j-ang_range + k-conn_range) * angle_scale / (2*ang_range);
                float side = remainder(atan2(dy_near, dx_near)-beta,2*M_PI);
                if (side < 0) {
                    d = -d;
                }
                if (isnan(r_max[j]) || (r * r_scale < r_max[j])) {
                    float de = (d - d_desired) /* / (std::max(r,1u) * r_scale) */;
                    // divide by (std::max(r,1) * r_scale) ?? 
                    if (d > d_safety) {
                        node_cost(r,j,k) = (de * de); 
                    } else {
                        // Adding obstacle repulsion
                        node_cost(r,j,k) = (de * de) + d_safety/(d+1e-10) - 1;
                    }
                } else {
                    // Behind the point cloud
                    node_cost(r,j,k) = NAN;
                }
#ifdef SAVE_OUTPUT
                printf("%3d,%3d,%d -> %6.2f %6.2f -> %6.2f (r_max %6.2f d %6.2f)\n",r,j,k,alpha,r*r_scale,node_cost(r,j,k),r_max[j],d);
                fprintf(fp,"%e %e %e %e %e %e\n",r*r_scale*cos(alpha),r*r_scale*sin(alpha),r*r_scale,alpha,d,node_cost(r,j,conn_range));
#endif
            }
#else
            if (isnan(r_max[j]) || (r * r_scale < r_max[j])) {
                float de = (d - d_desired) /* / (std::max(r,1u) * r_scale) */;
                // divide by (std::max(r,1) * r_scale) ?? 
                if (d > d_safety) {
                    node_cost(r,j) = (de * de); 
                } else {
                    // Adding obstacle repulsion
                    node_cost(r,j) = (de * de) + d_safety/(d+1e-10) - 1;
                }
            } else {
                // Behind the point cloud
                node_cost(r,j) = NAN;
            }
#ifdef SAVE_OUTPUT
            printf("%3d,%3d -> %6.2f %6.2f -> %6.2f (r_max %6.2f d %6.2f)\n",r,j,alpha,r*r_scale,node_cost(r,j),r_max[j],d);
            fprintf(fp,"%e %e %e %e %e %e\n",r*r_scale*cos(alpha),r*r_scale*sin(alpha),r*r_scale,alpha,d,node_cost(r,j));
#endif
#endif
        }
    }
#ifdef SAVE_OUTPUT
    fclose(fp);
#endif

}

std::list<cv::Point2f> RadialPlan::getOptimalPath(float K_initial_angle, float K_length, float K_turn, float K_dist)
{
    int dims[3] = {n_r, n_j, n_k};
    cv::Mat_<float> cell_value(3,dims, NAN);
    // For each cell we need to store a pointer to the coordinates of
    // its best predecessor. 
    cv::Mat_<cv::Vec3s> predecessor(3,dims);
    // The core of Dijkstra's Algorithm, a sorted heap, where the first
    // element is always the closer to the start.
    Heap heap;
    cell_value(0,ang_range,conn_range) = 0;
    for (int k = -conn_range;k<=conn_range;k++) {
        float alpha = (k-conn_range) * angle_scale / (2*ang_range);
        float cost = r_scale + K_initial_angle * alpha * alpha;
#ifdef SIGNED_DISTANCE
        cost += node_cost(1,ang_range+k,conn_range);
#else
        cost += node_cost(1,ang_range+k);
#endif
        if (!isnan(cost)) {
            heap.insert(Heap::value_type(cost, cv::Point3i(1,ang_range+k,conn_range)));
            cell_value(1,ang_range+k,conn_range) = cost;
            predecessor.at<cv::Vec3s>(1,ang_range+k,conn_range) = cv::Vec3s(0,ang_range,conn_range);
        }
    }
    while (!heap.empty()) {
        // Select the cell at the top of the heap
        Heap::iterator hit = heap.begin();
        // the cell it contains is this_cell
        cv::Point3i this_cell = hit->second;
        // and its score is this_cost
        float this_cost = cell_value(this_cell.x,this_cell.y,this_cell.z);
#ifdef SAVE_OUTPUT
        printf("Considering cell %3d, %3d, %3d, score %6.2f\n",
                this_cell.x,this_cell.y,this_cell.z,this_cost);
#endif
        // We can remove it from the heap now.
        heap.erase(hit);
        // Now see where we can go from this_cell
        for (unsigned int i=0;i<neighbours.size();i++) {
            cv::Point3i dest = this_cell;
            dest.x += neighbours[i].x;
            if (neighbours[i].x > 0) {
                dest.y += (this_cell.z-conn_range);
            }
            dest.z += neighbours[i].z;
#ifdef SAVE_OUTPUT
            printf("\tTo %d %d %d: ",dest.x,dest.y,dest.z);
#endif
            // WARNING: We're NOT wrapping around angles, because this would
            // depend on alpha_scale
            if (!isInGrid(dest)) {
                // outside the grid
#ifdef SAVE_OUTPUT
                printf("outside\n");
#endif
                continue;
            }
#ifdef SIGNED_DISTANCE
            float cell_cost = node_cost(dest.x,dest.y,dest.z);
#else
            float cell_cost = node_cost(dest.x,dest.y);
#endif
            if (isnan(cell_cost)) {
                // behind the point cloud
#ifdef SAVE_OUTPUT
                printf("behind\n");
#endif
                continue;
            }
            float trans_cost;
            if (neighbours[i].x == 0) {
                // Rotation on the spot
                float d_alpha = neighbours[i].z * angle_scale / n_j;
                trans_cost = K_turn * d_alpha * d_alpha;
            } else {
                trans_cost = transition_cost(this_cell.x,this_cell.z);
            }
            float new_cost = this_cost + trans_cost + (neighbours[i].x?cell_cost:0);
#ifdef SAVE_OUTPUT
            printf("T %6.2f C %6.2f N: %6.2f",trans_cost,cell_cost,new_cost);
#endif
            float cv = cell_value(dest.x,dest.y,dest.z);
            if (isnan(cv) || (new_cost < cv)) {
                // found shortest path (or new path), updating the
                // predecessor and the value of the cell
                predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                cell_value(dest.x,dest.y,dest.z) = new_cost;
                // And insert the selected cells in the map.
                if (dest.x < (n_r-1)) {
                    heap.insert(Heap::value_type(new_cost,dest));
                }
#ifdef SAVE_OUTPUT
                printf(" OK\n");
#endif
            } else {
#ifdef SAVE_OUTPUT
                printf(" NO\n");
#endif
            }
            // getchar();
        }
    }
    float best_potential = NAN;
    int best_j=0, best_k=0;
    int r_max = n_r - 1;
    for (unsigned int j=0;j<n_j;j++) {
        for (unsigned int k=0;k<n_k;k++) {
            if (isnan(cell_value(r_max,j,k))) {
                continue;
            }
            if (isnan(best_potential) || (cell_value(r_max,j,k) < best_potential)) {
                best_potential = cell_value(r_max,j,k);
                best_j = j; best_k = k;
            }
        }
    }

    std::list<cv::Point2f> lpath;
    if (isnan(best_potential)) {
        // No path found
        ROS_ERROR("No path found to desired r_max");
        return lpath;
    }
    // ROS_INFO("Planning completed");
    // Now extract the path by starting from goal and going through the
    // predecessors until the starting point
    cv::Point3i current(r_max,best_j,best_k);
    unsigned int count = 0;
#ifdef SAVE_OUTPUT
    FILE * fp = fopen("path","w");
#endif
    while (1) {
        float j_f = (current.y - ang_range)*angle_scale / (2*ang_range);
#ifdef SAVE_OUTPUT
        float k_f = (current.z - conn_range)*angle_scale / (2*ang_range);
#endif
        float x = current.x*r_scale*cos(j_f);
        float y = current.x*r_scale*sin(j_f);
#ifdef SAVE_OUTPUT
        printf("%d %d %d -> %.2f %.2f %.2f -> %.2f %.2f\n",current.x,current.y,current.z,
                current.x*r_scale, j_f, k_f, x, y);
        fprintf(fp,"%e %e %e\n",x,y,0.0);
#endif
        lpath.push_front(cv::Point2f(x,y));
        if (current.x == 0) {
            // Terminate when r == 0
            break;
        }
        cv::Vec3s p = predecessor(current.x,current.y,current.z);
        current.x = p[0]; current.y = p[1]; current.z = p[2];
        assert(lpath.size()<10000);
        count ++;
        assert(count < n_r * n_k);
    }
#ifdef SAVE_OUTPUT
    fclose(fp);
#endif

    return lpath;
}

