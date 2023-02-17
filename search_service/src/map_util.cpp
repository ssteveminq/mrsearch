#include "map_util.h"




Map_params::Map_params():xyreso(0.5),yawreso(0.2), xmin(-12.0),xmax(12.0),ymin(-15.0),
    ymax(12.0), sensor_range(10.0)
    {
        xw = int(round((xmax - xmin) / xyreso));
        yw = int(round((ymax - ymin) / xyreso));
    }
Map_params::Map_params(double maxx, double maxy, double minx, double miny):xyreso(0.05),yawreso(0.02), xmin(minx),xmax(maxx),ymin(miny),
    ymax(maxy), sensor_range(10.0)
    {
        xw = int(round((xmax - xmin) / xyreso));
        yw = int(round((ymax - ymin) / xyreso));
    }

Map_params::Map_params(double maxx, double maxy, double minx, double miny, double xy_reso):xyreso(xy_reso),yawreso(0.02), xmin(minx),xmax(maxx),ymin(miny),
    ymax(maxy), sensor_range(10.0)
    {
        xw = int(round((xmax - xmin) / xyreso));
        yw = int(round((ymax - ymin) / xyreso));
    }
    
void Map_params::calc_grid_map_config( double agent_x, double agent_y, double& minx, double& miny, double& maxx, double& maxy, double& xw, double& yw)
    {
    
        minx = agent_x - sensor_range;
        maxx = agent_x + sensor_range;
        miny = agent_y - sensor_range;
        maxy = agent_y + sensor_range;
        xw = int((maxx - minx) / xyreso);
        yw = int((maxy - miny) / xyreso);
    }


