/*
 * map_util.h
 */

#ifndef MAP_UTIL_H
#define MAP_UTIL_H

#include <math.h>

class Map_params
{
    public:
        Map_params();
        Map_params(double maxx, double maxy, double minx, double miny);
        Map_params(double maxx, double maxy, double minx, double miny, double xy_reso);
       
        void calc_grid_map_config( double agent_x, double agent_y, double& minx, double& miny, double& maxx, double& maxy, double& xw, double& yw);

        //precast
        double xyreso;
        double yawreso;
        double xmin;
        double ymin;
        double xmax;
        double ymax;
        double xw;
        double yw;
        double sensor_range;

};




#endif /* TK_SPLINE_H */
