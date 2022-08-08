# mocap_measurement

This program is used to compare a Gazebo's rigid body position with an AMCL estimation

# Installation

1. Clone in your workspace the repository:
```bash
my_ws$ mkdir src && cd src
my_ws/src$ git clone https://github.com/jmguerreroh/mocap_measurement.git
```

2. Install dependencies
```bash
my_ws/src$ vcs import < mocap_measurement/dependency_repos.repos
my_ws$ cd .. && rosdep install --from-paths src --ignore-src -r -y
```

3. Build workspace
```bash
my_ws$ source /opt/ros/<ros-distro>/setup.bash
my_ws$ colcon build --symlink-install
```

   - Ensure you have installed the Turtlebot3 packages to get the gazebo models
```bash
sudo apt install ros-<ros-distro>-turtlebot3*
```
   - Export gazebo models
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros-distro>/share/turtlebot3_gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/<your-user>/<directory>/install/gazebo_mocap_plugin/share/gazebo_mocap_plugin/models/
```

4. Run the simulator with “a marker”. Remember to use the “2D Pose Estimate” tool in Rviz2 (in toolbar) to set the robot position (it is around (X: -2, Y: -0.5, Yaw: 0.0) )
```bash
my_ws$ source install/setup.bash
my_ws$ ros2 launch mocap_measurement tb3_simulation_launch.py
```

   - Run gzclient if you want to see the simulation
```bash
mocap4ros2_ws$ gzclient
```


5. Run RQT Gui and load the MocapControl plugin under “Plugins -> MOCAP4ROS2 -> Mocap Control”
```bash
my_ws$ ros2 run rqt_gui rqt_gui --force-discover
```

6. Press the button “Start” in MocapControl and check that markers and rigid bodies being published:
```bash
ros2 topic echo /markers
ros2 topic echo /rigid_bodies
```

7. Run mocap_measurement:
```bash
ros2 run mocap_measurement measurement
```

   - Check "rb_pose" and "measurement" topics:
```bash
ros2 topic echo /rb_pose
ros2 topic echo /measurement
```