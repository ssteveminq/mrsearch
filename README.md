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
 -- You have to manually set the repo for "dir_path". 
```

#Simulation
 - roslaunch tmp multi_spot_ahg.launch
 - roslaunch search_service mtsp_search.launch
 - rosrun search_service multisearch_action_client.py
 
## Real Single Search Robot Experimeint


