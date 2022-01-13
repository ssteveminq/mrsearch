#include <ros/ros.h>
#include "spline.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>


using namespace std;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "spline_test");
    //ros::NodeHandle n;
    //ros::Rate loop_rate(1500);

    std::vector<double> xs;
    std::vector<double> ys;

    xs.push_back(2.0);
    xs.push_back(5.0);
    xs.push_back(7.5);
    xs.push_back(9.0);
    xs.push_back(10.5);
    xs.push_back(11.4);

    ys.push_back(-2.0);
    ys.push_back(0.0);
    ys.push_back(3.5);
    ys.push_back(2.0);
    ys.push_back(-2.5);
    ys.push_back(-8.4);

    tk::spline s;
    s.set_points(xs,ys);
    tk::spline2D spline2d(xs,ys);

    double sample_x, sample_y=0.0;
    for(int k(0); k<8;k++)
    {
        spline2d.calc_positions(k*3-3,sample_x, sample_y);
            std::cout<<"s: "<<k*3-3<<", x:"<<sample_x <<", y: "<<sample_y <<std::endl;
    }
    

    return 0;

}

