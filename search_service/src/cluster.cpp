
#include "ros/ros.h"
#include <numeric>
#include <math.h>
#include "cluster.h"



HCluster::HCluster(int num_label_, vector<double>& xs, vector<double>& ys, vector<geometry_msgs::Pose>& states, vector<double>& weights )
{
    num_label = num_label_;
    if(xs.size()==ys.size())
        {
            num_data = xs.size();
            m_xs.resize(num_data,0.0);
            m_ys.resize(num_data,0.0);
            for(int i(0);i<num_data;i++)
            {
                m_xs[i] = xs[i];
                m_ys[i] = ys[i];
            }
            m_labels.resize(num_data,0);
            m_centerx.resize(num_data,0.0);
            m_centery.resize(num_data,0.0);
            m_center_updated.resize(num_data,0);
            m_weights.resize(num_data,1.0);
            for(int j(0);j<weights.size();j++)
                m_weights[j]=1/(weights[j]+1);
        }
        else
        {
            cout<<"dimension wrong"<<endl;
            return;
        }
        std::default_random_engine generator;
        generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::uniform_int_distribution<int> distribution(0,num_label-1);
        for(int i(0); i< num_data; i++)
        {
            int tmp_label = distribution(generator);
            cout<<"tmp_label: "<<tmp_label<<endl;
            m_labels[i]=tmp_label;
        }

        //obstacles
        //std::vector<double> w1;
        //w1.push_back(4);
        //w1.push_back(-4.5);
        //w1.push_back(4.5);
        //obstacle_map.insert({0,w1});
        //
        //std::vector<double> w2;
        //w2.push_back(9.5);
        //w2.push_back(-4.2);
        //w2.push_back(4.2);
        //obstacle_map.insert({1,w2});

        //std::vector<double> w3;
        //w3.push_back(-5.);
        //w3.push_back(-7.5);
        //w3.push_back(7.5);
        //obstacle_map.insert({0,w3});

        //std::vector<double> w4;
        //w4.push_back(4.0); //
        //w4.push_back(-10.5);
        //w4.push_back(7.5);
        //obstacle_map.insert({0,w4});

        set_centers(states);
        run_clustering();
        for(int i(0); i< num_data; i++)
        {
            cout<<"(x,y) : "<<m_xs[i]<<", "<<m_ys[i]<<" , label: "<<m_labels[i]<<endl;
        }



}

int HCluster::get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y, 
    float p2_x, float p2_y, float p3_x, float p3_y)
{
    float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return 0; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return 0; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return 0; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return 0; // No collision
    // Collision detected
    //t = t_numer / denom;
    //if (i_x != NULL)
        //*i_x = p0_x + (t * s10_x);
    //if (i_y != NULL)
        //*i_y = p0_y + (t * s10_y);

    return 1;
}




//void HCluster::set_centers(std::vector<double>& centerx, std::vector<double>& centery)
void HCluster::set_centers(std::vector<geometry_msgs::Pose> states)
{
    //if(centerx.size()!=centery.size())
    //{
        //cout<<"wrong input dimension"<<endl;
        //return;
    
    //}

    int num_pts= states.size();
    for(int j(0);j<num_pts;j++)
    {
        m_centerx[j]=states[j].position.x;
        m_centery[j]=states[j].position.y;

        cout<<"center (x,y): "<<m_centerx[j]<<","<<m_centery[j] <<endl;
    }

}

//void HCluster::calc_num_labels()
//{

    //for(int j(0);j<num_label;j++)
        //x, y = self._get_labeled_x_y(label)
        //print(label , ": ", len(x))


//}




void HCluster::get_labeled_x_y(int target_label, vector<double>& rx,  vector<double>& ry)
{
    rx.clear();
    ry.clear();
    for(size_t i(0);i<num_data;i++)
    {
        if(m_labels[i]==target_label)
        {
            rx.push_back(m_xs[i]);
            ry.push_back(m_ys[i]);
        }
    }
}


void HCluster::update_clusters()
{


}

bool HCluster::check_obstacles(double x1, double x2, double y1, double y2)
{
    //check if there exist wall between two points
    ////wall extreme points 
    //wall along x coordinate , [fix coordinate(y), x1, x2]
    //wall along y coordinate , [fix coordinate(x), y1, y2]
    //p0: min _wall from obstacle_map  0=> x(second[1], second[0])  1=> (second[2], second[0])
    //p1: max _wall                    1=> x(second[0], second[1])  1=> (second[0], second[2])
    //p2: (x1,y1)
    //p3: (x2,y2)

   std::map<int, std::vector<double>>::iterator obiter=obstacle_map.begin(); 
   bool collision=false;
   for(obiter; obiter!= obstacle_map.end(); obiter++)
   {
       if(obiter->first==0)
       {
           if(get_line_intersection(obiter->second[1], obiter->second[0], obiter->second[2], obiter->second[0],
                   x1,y1,x2,y2))
               return true;
       }
       else{
           if(get_line_intersection(obiter->second[0], obiter->second[1], obiter->second[0], obiter->second[2],
                   x1,y1,x2,y2))
               return true;
       }
   }
   return false;


}


double HCluster::update_weightedclusters()
{
    double cost=0.0;
    double px, py =0.0;
    for(size_t i(0);i<num_data;i++)
    {
        px = m_xs[i];
        py = m_ys[i];
        std::vector<double> distance_set(num_label,0.0);
        //std::vector<double> dx(num_label,0.0);
        for(size_t j(0);j<num_label;j++)
        {
            distance_set[j]=sqrt(pow(m_centerx[j]-px,2)+pow(m_centery[j]-py, 2));
            if(check_obstacles(m_centerx[j], px, m_centery[j],py))
                distance_set[j]=4.0*distance_set[j];
            distance_set[j]=m_weights[j]*distance_set[j];
            //cout<<"distance_set[ "<<j<<"] : "<<distance_set[j]<<endl;
        }
    
        auto minIt = std::min_element(distance_set.begin(), distance_set.end());
        double minElement = *minIt;
        int min_idx = minIt -distance_set.begin();
        //cout<<"min_idx: "<<min_idx<<endl;
        m_labels[i]=min_idx;
        cost += minElement ;
    
        //cout<<"-------------------"<<endl;
    }
    return cost;



}
void HCluster::run_clustering()
    {
        double pre_cost = 10000.0;
        double cost=0.0;
        for(int i(0);i<MAX_LOOP;i++)
        {
            cost = update_weightedclusters();
            calc_centroid_selected();
            if(abs(pre_cost-cost)<cost_threshold)
            {
                cout<<"done"<<endl;
                break;
            }
            pre_cost = cost;
        }
    
    }



void HCluster::set_weightcenters(const std::vector<double>& weights)
{

    for(int i(0);i<weights.size();i++)
    {
        m_weights[i]=(1.0/weights[i]);
    }

}

void HCluster::calc_centroid_selected()
{
    for(int i(0);i<num_label;i++)
    {
        if(m_center_updated[i]==1)
        {
            vector<double> tmp_xs;
            vector<double> tmp_ys;
            get_labeled_x_y(i,tmp_xs, tmp_ys);
            int num_data_ = tmp_xs.size();
            if(num_data_>0)
            {
                auto sum_of_elems_x = std::accumulate(tmp_xs.begin(), tmp_xs.end(), 0);
                auto sum_of_elems_y = std::accumulate(tmp_xs.begin(), tmp_xs.end(), 0);
                m_centerx[i]= sum_of_elems_x/num_data_;
                m_centery[i]= sum_of_elems_y/num_data_;
            }
        }
    }
}



