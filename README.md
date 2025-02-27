# CS-499-RoboCourier
This is a project which adds to and utilizes Turtlebot4 with ros2 and Gazebo to demonstrate indoor courier navigation.

#### Rescoures used
Turtlebot4 - https://github.com/turtlebot/turtlebot4
            - https://github.com/turtlebot/turtlebot4_simulator
            
AWS hospital world - https://github.com/aws-robotics/aws-robomaker-hospital-world


# Features
## Keepout Zones
An easy package to launch keepout zones independent of the launch file. Useful for handeling dynamic events that could happen within the environment i.e. a hallway or room closed down or an area has become permanently off limits.

#### Hospital world generated with SLAM
![image](https://github.com/user-attachments/assets/852869bc-79e2-4d84-bc19-f455c4dc8f52)

#### Edited with GIMP for keepout zones
![image](https://github.com/user-attachments/assets/4b413a49-f0b9-4211-af8a-300988ff8c52)

### To launch keepout zone
`ros2 run courier_navigation keepout.launch.py filter:=path/to/keepout`

##### Notes
- When doing keepout zones the Turtlebot4 launch is edited to include the boolean parameter 'keepout' default=false. This is not to launcht he keepout Node itself, but to launch Nav2 with courier_navigationconfig/nav2_with_keepout.yaml
`ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py keepout:=true`


## Client/Service Node for Waypoints
The services/ package contains nodes which integrate with the local webserver for an users to request pickup/delivery of the courier. 

## To launch service and request a pickup/delivery

Launch the service node
`ros2 run servies map_to_pose_service --ros-args -p waypoints:=/path/to/waypoints # default hospital.yaml`

waypoints is the path to a .yaml file of waypoints you have saved once you have a map generated with SLAM
The default location for waypoints is /services/service_materials/ 


Launch the client to issue a request
`ros2 run services map_to_pose_client a101 b201 w1`
This will then navigate to rooms locations which are named in the yaml file. If a location doesn't exist it will simply discard the room and go to the next waypoint


###### examples
![image](https://github.com/user-attachments/assets/5a271ff7-66d3-47e6-86e6-b4e24aec7246)






![image](https://github.com/user-attachments/assets/334d00f2-bcb7-4a8a-87cb-ded157601501)






## Installing ROS2 Jazzy & Gazebo Harmonic
**_Important_** This will only work for Ubuntu 24.04 or later

**For a quick install** run `setup_ros2_gazebo.bash` located in `init_setp/`
`source init_setup/setup_ros2_gz.bash`

This will add `source /opt/ros/jazzy/setup.bash` .bashrc so each terminal opened thereafter has access to the base ROS2 commands and packages that come with installing ROS2

**To install manually**
More info on ROS2 - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#
More info on Gazebo - https://gazebosim.org/docs/latest/install_ubuntu/

## Installing Turtlebot4 Simulation
Note - https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html
This project has a cloned package that may not be up to date with the current version of turtlebot for as of 12/13/2024


## Note on create3
There might be a necessity to clone create3 depending on if it has been updated since 12/13/2024.






