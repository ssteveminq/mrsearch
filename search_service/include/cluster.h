
#ifndef CLUSTER_H
#define CLUSTER_H 

#include <vector>
#include <numeric>
#include <random>
#include <map>
#include <math.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "geometry_msgs/PoseStamped.h"
#include <chrono>

using namespace std;
#define MAX_LOOP 30 
#define cost_threshold 0.025

class HCluster
{

    public:
    HCluster(){};
    HCluster(int num_label_, vector<double>& xs, vector<double>& ys, vector<geometry_msgs::Pose>& states, vector<double>& weights );


    public:
    vector<double> m_xs;
    vector<double> m_ys;
    vector<double> m_weights;
    vector<int> m_labels;
    int num_data;
    int num_label;
    vector<double> m_centerx;
    vector<double> m_centery;
    vector<bool> m_center_updated;

    void set_centers(std::vector<geometry_msgs::Pose> states);
    void set_weightcenters(const std::vector<double>& weights);
    void get_labeled_x_y(int target_label, vector<double>& rx,  vector<double>& ry);
    void update_clusters();
    double update_weightedclusters();
    void calc_centroid_selected();
    void run_clustering();
    bool check_obstacles(double x1_, double x2, double y1, double y2);
    int get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, 
        float p2_x, float p2_y, float p3_x, float p3_y);
   //wall => key: x[0] or y[1] value : coordinate, range(2,5)
   //ex) {1, {2,6} } ==>y coordinate  wall which comes from 2 to 6
    std::map<int, std::vector<double> > obstacle_map;



};


#endif
