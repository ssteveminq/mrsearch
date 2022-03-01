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


## Turtlebot dependencies

- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs

# Three Turtlbebot
Gazebo world
```
export TURTLEBOT3_MODEL=burger
roslaunch tmp three_turtle_home_outdoor.launch
roslaunch search_service pose_pub.launch

```
Search Server
```
roslaunch search_service mtsp_search_three_turtle.launch
```

Set_polygon
```
rosrun search_service multisearch_action_client.py
```

Once all paths are obtained

```
roslaunch search_service pathfollow.launch
```

# Three Turtlbebot with Replanning
Gazebo world
```
export TURTLEBOT3_MODEL=burger
roslaunch tmp three_turtle_home_outdoor.launch
roslaunch search_service pose_pub.launch

```
Search Server
```
roslaunch search_service mtsp_search_three_turtle.launch
```

Set_polygon
```
rosrun search_service multisearch_action_client.py
```

Once all paths are obtained

```
roslaunch search_service pathfollow.launch
roslaunch search_service pathfollowclient.launch
```
To Re-Plan, kill both of the pathfollow launch files, and run 
```
rosrun search_service replan_multisearch_client.py
```
If failure occurs, remove the corresp. line from the pathfollow launch files and then re launch once replan client is done.
```

```

# Predcition!
```
rosrun gmm human_belief
```
To change prediction
```
rosrun gmm human_belief prediction_client.py
rosrun gmm human_belief prediction_client2.py
```

# Five Turtlbebot
Gazebo world
```
export TURTLEBOT3_MODEL=burger
roslaunch tmp five_turtles_disaster.launch
roslaunch search_service pose_pub.launch

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


