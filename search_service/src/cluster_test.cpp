#include <ros/ros.h>
#include "cluster.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>


using namespace std;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "tsp_test");
    //ros::NodeHandle n;
    //ros::Rate loop_rate(1500);

    std::vector<double> xs;
    std::vector<double> ys;
    int num_samples = 50;

    std::default_random_engine ngenerator;
    ngenerator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> ndistribution(-15.0,15.0);
    for(size_t i(0);i<num_samples;i++)
    {
    
        double tx = ndistribution(gen);
        xs.push_back(tx);
        double ty = ndistribution(gen);
        ys.push_back(ty);
    }

    /*
    xs.push_back(0.5);
    xs.push_back(2.0);
    xs.push_back(5.0);
    xs.push_back(7.5);
    xs.push_back(9.0);
    xs.push_back(10.5);
    xs.push_back(11.4);
    xs.push_back(-5.5);
    xs.push_back(15.5);
    xs.push_back(-1.4);
    xs.push_back(-7.4);
    xs.push_back(-4);
    xs.push_back(-9.4);
    xs.push_back(2);

    ys.push_back(9.2);
    ys.push_back(0.0);
    ys.push_back(-3.5);
    ys.push_back(2.0);
    ys.push_back(-2.5);
    ys.push_back(-8.4);
    ys.push_back(13.5);
    ys.push_back(2.0);
    ys.push_back(-12.5);
    ys.push_back(2.5);
    ys.push_back(3.8);
    ys.push_back(-13.0);
    ys.push_back(-2.5);
    ys.push_back(12.5);
    */


    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<> distribution(-11.0,8.0);

    std::vector<double> weights;
    std::vector<geometry_msgs::Pose> Posevec;
    for(int j(0);j<4;j++)
    {

        geometry_msgs::Pose tmp_pos;
        double tmp_x = distribution(generator);
        double tmp_y = distribution(generator);

        tmp_pos.position.x = tmp_x;
        tmp_pos.position.y = tmp_y;
        Posevec.push_back(tmp_pos);
    
        weights.push_back(1.0);
    }
  
    //HCluster test_cluster;
    HCluster test_cluster(4, xs,ys, Posevec, weights);

    std::ofstream outfile;
    outfile.open("cluster_sol.csv", std::ofstream::out | std::ofstream::trunc);
    for(int i(0);i<xs.size();i++)
    {
        outfile<<i<<"\t"<< xs[i]<<"\t"<<ys[i]<<"\t"<<test_cluster.m_labels[i]<<"\n";
    }
    outfile.close();
 
    std::ofstream outfile2;
    outfile2.open("cluster_con.csv", std::ofstream::out | std::ofstream::trunc);
    for(int i(0);i<Posevec.size();i++)
    {
        outfile2<<i<<"\t"<< Posevec[i].position.x<<"\t"<<Posevec[i].position.y<<"\n";
    }
    outfile2.close();
 


    return 0;

}

