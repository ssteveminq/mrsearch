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
 
## Documentation
See [API wiki](https://github.com/ssteveminq/mrsearch/wiki/API)


## Third Party Code
[1] - Biswas, Joydeep, and Manuela Veloso. "Episodic non-markov localization: Reasoning about short-term and long-term features."2014 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2014.
Dependencies
1. sudo apt install qt5-default libqt5websockets5-dev libgoogle-glog-dev libgflags-dev cd catkin_ws/src/a1_autonomy/enml && ./InstallPackages

Build the Code
1. cd mrsearch/third_party
2. export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH 
3. cd amrl_msgs && export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH 
4. make
5. cd .. 
6. make 
7. catkin build




## Visual coverage path planning
path information is saved in path.yaml file in search_service/config. To load this path information you need to command.
```
cd ~/workspaces/test_ws/src/mrsearch/search_service
```
Check the (path_file_name) in line 61 in mrsearch/search_serivce/script/load_paths.py
To run this script, you can type
```
rosrun search_service load_paths.py
```





