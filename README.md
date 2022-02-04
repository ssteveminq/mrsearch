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
 - Change some parameters in search_service/launch/mtsp_search.launch
 

#Simulation
- Setup the Gazebo world with two spots 
```
roslaunch tmp multi_spot_ahg.launch
```
- Run the nodes for multi robot search (Server)
```
roslaunch search_service mtsp_search.launch
```
- Run action client for multisearch (Client)
``` 
rosrun search_service multisearch_action_client.py
```
 
## Documentation
See [API wiki](https://github.com/ssteveminq/mrsearch/wiki/API)


