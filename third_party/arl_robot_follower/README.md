# arl_robot_follower

arl_robot_follower is a ROS package which supports an action server to make a robot follow a target. It is intended for use with the Army Research Laboratory's (ARL) Phoenix navigation stack.

## Server API

 ROS Subscribers
    follow_target [geometry_msgs/PoseStamped] - Listens to the pose of the follower robot.
 
 ROS Action Clients
    goto_region [arl_nav_msgs/GoToRegionAction] - Sends the navigation commands to the follower robot.
 
 ROS Action Servers
    follow_target [arl_robot_follower/FollowTargetAction] - The action provided by this server. Tells the follower robot
                                                            to begin following a target frame.

## Robot Follower Behavior

The ```robot_follower_server``` node provides an action server to support robot following behavior. This allows you to command a robot to follow behind a specific reference frame at a given distance.

### Usage

The run the server:
```rosrun arl_robot_follower robot_follower_server```

Connect the ```follower_pose``` topic to the pose of the follower robot.
Connect the ```goto_region``` action client to a navigation server for the follower robot.
Send goals to the ```follow_target``` action server to activate the follow behavior.

### arl_robot_follower/FollowTargetAction

This action specifies a minimum and maximum distance to maintain to a target reference frame. The follower will try to stay behind the target frame (meaning  the negative x region).

## Target Inspection Behavior

The ```follower_inspection_server``` node provides an action server to support close inspection behavior. This allows you to command a robot to approach a specific point and collect data, including a photograph and pointcloud.

### Usage

The run the server:
```rosrun arl_robot_follower follower_inspection_server```

Connect the ```image``` topic to the color feed of the robot camera.
Connect the ```pointcloud``` action client to a LIDAR device on the robot.
Send goals to the ```inspect_target``` action server to activate the inpsection behavior.

### Inspection Assumptions

<ol>
<li>The front of the target is perpendicular to ground.</li>
<li>The item is no more than 2m above ground level. This is measured from the center of the detection area, i.e. an AR tag, human face, etc.</li>
<li>The inspection camera is fixed relative to the robot base link, and pointed parallel to the ground.</li>
<li>Inspection camera is oriented where Z is forward, Y is up.</li>
</ol>

## Supervisor Node

The ```supervisor``` node is the core routine of the UTDD autonomous recon demo. It directs the leader and follower vehicles to execute the demo steps, and activates the target-of-interest inspection behavior when one is detected.

### Usage

The run the server:
```rosrun arl_robot_follower supervisor lead_robot_name follow_distance```

```lead_robot_name``` - The name of the leader robot. This robot must have a joint named \<lead_robot_name\>/base_link.  
```follow_distance``` - The distance at which the follower robot should try to stay from the leader.
