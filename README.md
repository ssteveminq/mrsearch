# person_finding
## prerequisites
```
sudo apt-get install ros-melodic-navigation
sudo apt-get install ros-melodic-bfl
sudo apt-get install ros-melodic-geometry2
pip install shapely
pip install pandas
pip install scipy
```
## Dependencies
- frontier_exploration 
- https://github.com/ssteveminq/frontier_exploration

## Run the search (For A1) NUC
- roslaunch unitree_legged_real a1_search.launch
- rosrun search_service map_manager_single.py
- Checkpoints (rviz)
  -- Localization / costmap
  -- search_map
- rosrun search_service search_action_client.py
- rosrun search_service a1_move

## Run the search (For HSR)
- Openvpn setting
- HSR(Robot)
```
 source amrl_ws
 rosnode kill laser_2d_localizer 
 rosrun tmc_grid_map_server grid_map_server ahg5_map.yaml
 ```
- Backpack
```
hsrb_mode 
source amrl_ws
add amrl repo path to ros_packagepath
roslaunch tmp hsrb_noenml.launch
webviz or vectorydisplay
```
- HSR(Robot)
After running all nodes above,
```
rosrun search_service hsr_search_server_noenml.py
rosrun search_service search_action_client.py
rosrun search_service hsr_movenode.py

```

#Simulation
 - roslaunch unitree_gazebo a1_sim.launch
 - roslaunch unitree_gazebo a1_search.launch
 - vector display or webviz
 - robot state publisher (correct odom)
 
 
 - Updated 25th Nov- 
## Real Single Search Robot Experimeint

1. HSR
 - ssh to zilker
 ```
 rosnode kill /laser_2d_localizer
 roslaunch hsrb_single_search.launch
 cdam && ./bin/websocket_hsr
 rosrun search_service singlesearch_server_hsr
 rosrun search_service hsr_movenode
 
 ```


## Real Multi robot experiment (Using Multi Master )
1. HSR Setup
 - ssh to zilker ( ssh zilker@zilker-hsrb.local)
 - enable multi-cast (enable_mc) 
```
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts" && sudo service procps restart
```
 - You can check with the multi cast status: 0 (should be 0, not 1) (cat /proc/sys/net/ipv4/icmp_echo_ignore_broadcasts) 
 - unset ROS_IP ( This ROS_IP is set up by villa_tools for using other wired network (backpack, knapsack, or TK1)
 - stop openvpn service
 - stop and restart docker service (roscore service & robot service)
  
  ```
export ROS_MASTER_URI=http://zilker-hsrb:11311
unset ROS_IP
service openvpn stop
sudo systemctl stop docker.hsrb.robot.service
sudo systemctl stop docker.hsrb.roscore.service
sudo systemctl start docker.hsrb.roscore.service
sudo systemctl start docker.hsrb.robot.service
```
 - start HSR ( tourn on Emergency stop on HSR) => HSR should stand up with orange lights.
 - log on to hsr-user account ( ssh hsr-user@zilker-hsrb.local or Su - hsr-user (from Zilker account)
 - source amrl_ws
 - epa1
 
 ```
rosnode kill /laser_2d_localizer
export ROS_PACKAGE_PATH=/home/hsr-user/workspaces/amrl_ws/src/a1_autonomy:/home/hsr-user/workspaces/amrl_ws/src/a1_autonomy/amrl_msgs:/home/hsr-user/workspaces/amrl_ws/src/a1_autonomy/webviz:$ROS_PACKAGE_PATH
roslaunch tmp hsrb_ahg_nav.launch
```
(new terminal)
```
cd ~/workspaces/amrl_ws/src/a1_autonomy
./bin/websocket_hsrb
```
- open webviz html ( IP: 192.168.2.156) and connet
- localize with Set Pose
- run multi master disovery node
- After checking that HSR master discovers the Nuc's master, run sync node
```
unset ROS_IP
rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1
rosrun master_sync_fkie master_sync
```


2. A1 Setup
```
ssh amrl@192.168.2.194
sudo sh -c "echo 0 >/proc/sys/net/ipv4/icmp_echo_ignore_broadcasts"
sudo service procps restart
export ROS_MASTER_URI=http://192.168.2.194:11311
export ROS_IP=192.168.2.194
roscore
x86connect && sudo su && roslaunch unitree_legged_real a1_enml_mas.launch
```
Velodyne (another terminal)
```
puckconnect
epa1
roslaunch unitree_legged_real a1_visual.launch
```
Localization(another terminal)
```
cdam
./bin/websocket_a1
```
Other ROS Stuff(another terminal)
```
roslaunch unitree_legged_real a1_search.launch
roslaunch state_lattice_planner slp_server.launch
rosrun search_service a1_move_click
```

