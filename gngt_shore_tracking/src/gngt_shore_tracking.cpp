
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <laser_geometry/laser_geometry.h>
#include <cgnuplot/CGnuplot.h>

#include <vq2.h>

#define N_STEP_FROZEN 20
#define N_STEP_CHANGING 3
#define TARGET  1   // Adjust to adjust neuron creation rate
#define CONFIDENCE .75
#define EVOLUTION_DECISION_THRES 3
#define DELTA .75
#define EVOL_LOWPASSCOEF .4
#define EVOL_LOWPASSMARGIN .2




class GNGTShoreTracking {
    protected:
        
        ros::NodeHandle nh;
        ros::Subscriber imu_sub;
        ros::Subscriber pc_sub;
        ros::Subscriber ls_sub;
        ros::Publisher marker_pub;

        double min_dt;
        double target_value;
        double confidence_value;
        int step_frozen;
        int step_changing;

        struct PointOp {
            void 	raz (pcl::PointXYZ &v) {
                v.x = v.y = v.z = 0.0;
            }

            void 	add (pcl::PointXYZ &v, const pcl::PointXYZ &w) {
                v.x += w.x;
                v.y += w.y;
                v.z += w.z;
            }

            void 	div (pcl::PointXYZ &v, double coef) {
                v.x /= coef;
                v.y /= coef;
                v.z /= coef;
            }

            void 	mul (pcl::PointXYZ &v, double coef) {
                v.x *= coef;
                v.y *= coef;
                v.z *= coef;
            }
        };

        typedef vq2::algo::gngt::Unit<pcl::PointXYZ>  GNGTUnit;
        typedef vq2::temporal::Unit<GNGTUnit> Unit; // Adds .speed()
        // Do not forget the vertex initialization functor (3rd argument).
        typedef vq2::Graph<Unit,char,Unit::copy_constructor>    Graph;
        typedef Graph::vertex_type                 Vertex;
        typedef Graph::edge_type                 Edge;
        typedef Graph::ref_vertex_type             RefVertex;
        typedef Graph::ref_edge_type             RefEdge;

        // For finding the winner in the GNGT algorithm, we need some
        // similarity measure. Here, let us use the squared euclidian
        // distance.
        class Similarity {
            public:
                typedef pcl::PointXYZ value_type;
                typedef pcl::PointXYZ sample_type;

                double operator()(const value_type& arg1,
                        const sample_type& arg2) {
                    double dx = arg1.x - arg2.x;
                    double dy = arg1.y - arg2.y;
                    return dx*dx + dy*dy;
                }
        };
        typedef vq2::unit::UnitToSampleSimilarity<Unit,Similarity> UnitSimilarity;
        
        // The learning process.
        class Learn {
            public:
                typedef pcl::PointXYZ sample_type;
                typedef pcl::PointXYZ weight_type;
                void operator()(double coef,
                        weight_type& prototype,
                        const sample_type& target) {
                    prototype.x += coef * (target.x - prototype.x);
                    prototype.y += coef * (target.y - prototype.y);
                }
        };
        typedef vq2::unit::Learn<Unit,Learn> UnitLearn;

        // This is the parameter set for GNG-T
        class Params {
            public:
                int ageMax_;
                double firstLearningRate_;
            
            public:
                Params() : ageMax_(20), firstLearningRate_(0.01) {}
                int ageMax(void)                {return ageMax_;}
                double firstLearningRate(void)  {return firstLearningRate_;}
                double secondLearningRate(void) {return .2*firstLearningRate();}
                double lambda(void)             {return .001;}
                double infinity(void)           {return 1e12;}
        };

        // A value extraction functor.... identity here.
        static double value_of(double x) {return x;}
        
        class Evolution {
            public:

                std::vector<double> disto_distrib;
                double min,max,target,delta,lowpass_coef,lowpass_margin;
                bool first_run;
                size_t nb_samples;

                Evolution() : 
                    target(TARGET),delta(DELTA),
                    lowpass_coef(EVOL_LOWPASSCOEF),lowpass_margin(EVOL_LOWPASSMARGIN), first_run(true) {}

                void setParameters(double tgt,double dlta, double lowpass, double margin) {
                    target = tgt;
                    delta = dlta;
                    lowpass_coef = lowpass;
                    lowpass_margin = margin;
                }

                void setNbSamples(size_t nb_spls) {
                    nb_samples = nb_spls;
                }

                void clear(void) {
                    disto_distrib.clear();
                }

                void operator+=(double value) {disto_distrib.push_back(value);}

                int operator()() {
                    int res = 0;
                    double a,b;
                    vq2::proba::shortest_confidence_interval(disto_distrib.begin(),
                            disto_distrib.end(),
                            value_of,
                            delta,a,b);
                    if(first_run) {
                        min = a;
                        max = b;
                        first_run = false;
                    }
                    else {
                        min += lowpass_coef*(a-min);
                        max += lowpass_coef*(b-max);
                    }

                    double width;

                    width = (max-min);
                    double _min = min + lowpass_margin*width;
                    double _max = min + (1-lowpass_margin)*width;

                    if(nb_samples*target< _min)
                        res = 1;
                    else if(nb_samples*target> _max)
                        res = -1;
                    return res;
                }
        };


        Graph g;
        Params           params;
        Similarity       distance;
        UnitSimilarity   unit_distance;
        Learn            learn;
        UnitLearn        unit_learn;
        PointOp          op;
        Evolution        evolution;
        ros::Time        previous;
        std_msgs::Header      header;
        pcl::PointCloud<pcl::PointXYZ> pointCloud;
        visualization_msgs::MarkerArray markers;
        laser_geometry::LaserProjection projector;
        sensor_msgs::Imu imu;
        int num_scans;
        cgnuplot::CGnuplot plot;


        void imuCallback(const sensor_msgs::ImuConstPtr msg)  {
            imu = *msg;
        }

        void laserScanCallback(const sensor_msgs::LaserScanConstPtr msg)  {
            if (previous.toSec() < 1.0) {
                previous = msg->header.stamp;
                ROS_INFO("Initialised GNGT tracker");
                return;
            } 
            header = msg->header;
            sensor_msgs::PointCloud2 ros_cloud;
            projector.projectLaser(*msg, ros_cloud);
            pcl::fromROSMsg(ros_cloud, pointCloud);
            run_gngt();
        }

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg)  {
            if (previous.toSec() < 1.0) {
                previous = msg->header.stamp;
                ROS_INFO("Initialised GNGT tracker");
                return;
            } 
            header = msg->header;
            pcl::fromROSMsg(*msg, pointCloud);
            run_gngt();
        }

        void run_gngt() {
            pcl::PointXYZ omega(imu.angular_velocity.x,
                    imu.angular_velocity.y,imu.angular_velocity.z);
            DisplayVertex        display_vertex(omega,header,markers,min_dt+0.1);
            DisplayEdge        display_edge(header,markers,min_dt+0.1);
            double dt = (header.stamp - previous).toSec();
            if (dt < min_dt) {
                return;
            }
            previous = header.stamp;
            ros::Time t1 = ros::Time::now();
            vq2::temporal::tick(g,op,dt); // Here, speeds are updated. // TODO, dt = 1.0
            for (int i=0;i<step_frozen;++i) {
                make_epoch(false,params,g,unit_distance,unit_learn,op,evolution);
            }
            for (int i=0;i<step_changing;++i) {
                make_epoch(true,params,g,unit_distance,unit_learn,op,evolution);
            }
            ROS_INFO("GNGT optimization: %.1fms",1000. * (ros::Time::now()-t1).toSec());
            // Let us display the graph
            markers.markers.clear();
            g.for_each_edge(display_edge);
            g.for_each_vertex(display_vertex);
            marker_pub.publish(markers);

#if 0
            FILE * fp = fopen("vel.csv","w");
            for (size_t i=0;i<display_vertex.getVelocities().size();++i) {
                fprintf(fp,"%d %e %e\n",(int)i,
                        display_vertex.getVelocities()[i].x,
                        display_vertex.getVelocities()[i].y);
            }
            fclose(fp);
            plot.plot("set grid ; plot [-1:2][-1:1] \"vel.csv\" u 2:3 w p");
#endif
        }
        
        void make_epoch(bool growing, Params&           p, 
                Graph&            g, 
                UnitSimilarity&   distance,
                UnitLearn&        learn,
                PointOp&         op,
                Evolution&        evolution) {
            pcl::PointCloud<pcl::PointXYZ>::const_iterator iter,end;
            std::random_shuffle(pointCloud.begin(),pointCloud.end());
            evolution.setNbSamples(num_scans);
            vq2::algo::gngt::open_epoch(g,evolution);
            for(iter=pointCloud.begin(),end=pointCloud.end(); iter != end; ++iter) {
                vq2::algo::gngt::submit(p,g,distance,learn,*iter,growing);
            }
            vq2::algo::gngt::close_epoch(p,g,learn,evolution,growing);

            // We notify the end of frame for speed management.
            vq2::temporal::frame(g,op); 
        }
        // This is a loop functor class.
        class DisplayEdge {
            protected:
                const std_msgs::Header & header;
                visualization_msgs::MarkerArray & markers;
                double lifetime;
            public:
                DisplayEdge(const std_msgs::Header & h,visualization_msgs::MarkerArray & m, double lt):header(h),markers(m),lifetime(lt) {}
                bool operator()(Edge& e) {
                    pcl::PointXYZ p1, p2;
                    p1 = (*(e.n1)).value.prototype();
                    p2 = (*(e.n2)).value.prototype();

                    visualization_msgs::Marker marker;
                    marker.header = header;
                    marker.ns = "gngt_edges";
                    marker.id = markers.markers.size();
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.scale.x = 0.1;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.points.resize(2);
                    marker.points[0].x = p1.x;
                    marker.points[0].y = p1.y;
                    marker.points[1].x = p2.x;
                    marker.points[1].y = p2.y;
                    marker.lifetime = ros::Duration(lifetime);

                    markers.markers.push_back(marker);

                    return false; // the element should not be removed.
                }
        };

        // This is a loop functor class.
        class DisplayVertex {
            protected:
                const std_msgs::Header & header;
                pcl::PointXYZ omega;
                std::vector<pcl::PointXYZ> velocity;
                visualization_msgs::MarkerArray & markers;
                double lifetime;
            public:
                DisplayVertex(const pcl::PointXYZ & omega, const std_msgs::Header & h,
                        visualization_msgs::MarkerArray & m, double lt):header(h),omega(omega),markers(m),lifetime(lt) {}
                const std::vector<pcl::PointXYZ> getVelocities() const {
                    return velocity;
                }

                bool operator()(Vertex& n) { 
                    pcl::PointXYZ p,v;
                    // printf("%.3f\n", n.value.e / n.value.n);
                    p = n.value.prototype();
                    visualization_msgs::Marker marker;
                    marker.header = header;
                    marker.ns = "gngt_vertices";
                    marker.id = markers.markers.size();
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = p.x;
                    marker.pose.position.y = p.y;
                    marker.pose.position.z = 0.0;
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.1;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.lifetime = ros::Duration(lifetime);

                    markers.markers.push_back(marker);

                    v = n.value.speed();
                    marker.ns = "gngt_speed";
                    marker.id = markers.markers.size();
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = 0.0;
                    marker.pose.position.y = 0.0;
                    marker.pose.position.z = 0.0;
                    marker.scale.x = 0.10;
                    marker.scale.y = 0.05;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.lifetime = ros::Duration(lifetime);
                    marker.points.resize(2);
                    marker.points[0].x = p.x;
                    marker.points[0].y = p.y;
                    marker.points[1].x = p.x-v.x;
                    marker.points[1].y = p.y-v.y;

                    markers.markers.push_back(marker);

                    // Not working, probably because omega is not correct in
                    // the current test bags. To be evaluated on-board
                    velocity.push_back(pcl::PointXYZ(
                                v.x - omega.z*p.y,
                                v.y + omega.z*p.x,
                                0.0 ));

                    return false; // the element should not be removed.
                }
        };


    public:

        GNGTShoreTracking()  : 
            nh("~"), target_value(TARGET), confidence_value(CONFIDENCE), 
            unit_distance(distance), unit_learn(learn) {
                double delta_value, lowpass_coef, lowpass_margin;

                nh.param("num_scans",num_scans,1);
                nh.param("min_dt",min_dt,0.0);
                nh.param("target",target_value,(double)TARGET);
                nh.param("delta",delta_value,(double)DELTA);
                nh.param("lowpass_coef",lowpass_coef,(double)EVOL_LOWPASSCOEF);
                nh.param("lowpass_margin",lowpass_margin,(double)EVOL_LOWPASSMARGIN);
                nh.param("step_frozen",step_frozen,(int)N_STEP_FROZEN);
                nh.param("step_changing",step_changing,(int)N_STEP_CHANGING);
                nh.param("ageMax",params.ageMax_,params.ageMax_);
                nh.param("firstLearningRate",params.firstLearningRate_,params.firstLearningRate_);

                evolution.setParameters(target_value,delta_value,lowpass_coef,lowpass_margin);

                ls_sub = nh.subscribe("laserscan",1,&GNGTShoreTracking::laserScanCallback,this);
                pc_sub = nh.subscribe("pointcloud",1,&GNGTShoreTracking::pointCloudCallback,this);
                imu_sub = nh.subscribe("imu",1,&GNGTShoreTracking::imuCallback,this);
                marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers",1);
        }
};

int main(int argc, char *argv[]) 
{
    ros::init(argc,argv,"gngt_shore_tracking");
    GNGTShoreTracking tracker;

    ros::spin();
    return 0;
}

