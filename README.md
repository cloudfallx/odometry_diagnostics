# ODOM_PLOTTER

This tool can be used to subscribe to an odometry topic in ROS2 and display the euclidean distance of the robot with respect to `map` and `odom` frames. By default, the last 20 seconds of the data recieved is used to calculate accuracy of `odom` frame with respect to `map` frame.

After cloning the repo and building with `colcon build`, source the workspace and run:
```
ros2 launch odometry_diagnostics odom_plotter.py use_sim_time:=true
```
This will launch the visualizer tool.
