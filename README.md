# Multi-Robot-Search

## Environment
- Tested in Ubuntu 18.04 , ROS Melodic
## prerequisites 
```
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-bfl
sudo apt-get install ros-melodic-geometry2
```
## Dependencies
- SPOT simulation environment
- https://github.com/ssteveminq/search_sim


### Parameters
 - Configure launch parameters in search_service/launch/mtsp_search.launch
 

#Simulation
- Run the Gazebo world with two Spot quadrupeds (AHG world - 3 agents)
```
roslaunch tmp three_spot_ahg.launch
```
or (Disaster world - 2 agents)
```
roslaunch tmp multi_spot_disaster.launch
```

- Run the nodes for multi robot search (Server)
```
roslaunch search_service mtsp_search.launch
```
- Set search region (polygon) using rviz tools ( clicked point. Refer this [wiki](https://github.com/ssteveminq/mrsearch/wiki/API#5-demo-with-gazebo-simulation))
- Run action client for multisearch (Client)
``` 
rosrun search_service multisearch_action_client.py
```

#Five Turtlbebot
Gazebo world
```
roslaunch tmp five_turtles_disaster.launch
```
Search Server
```
roslaunch search_service mtsp_search.launch
```

Set_polygon
```
rosrun search_service multisearch_action_client.py
```

Once all paths are obtained

```
roslaunch search_service pathfollow.launch
```


 
## Documentation
See [API wiki](https://github.com/ssteveminq/mrsearch/wiki/API)


