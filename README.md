# ROB498: Take Cover

This is the repo for team Take Cover for ROB498


# Flight Exercise 2

 1. Vicon: make sure ROS_DOMAIN_ID = 1; ask TA to turn on vicon system;
 2. `ros2 topic echo /vicon/ROB498_Drone/ROB498_Drone`
 3. MAVROS: go to directory where mavros.launch.py is located (haven't made it a package yet), `ros2 launch mavros.launch.py`
 4. `python3 comm_node.py`
 5. `ros2 service list -t` --> then can just call the services
