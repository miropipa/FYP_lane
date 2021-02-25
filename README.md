# Autonomous Clean Project

SS47816



### Dependencies

**TODO**

```bash
sudo apt-get install ros-melodic-rospack ros-melodic-catkin ros-melodic-mrt-cmake-modules
sudo apt-get install libboost-dev libeigen3-dev libgeographic-dev libpugixml-dev libpython-dev libboost-python-dev python-catkin-tools

sudo apt install ros-melodic-uuid-msgs
sudo apt install ros-melodic-unique-id

sudo apt install ros-melodic-grid-map-cv ros-melodic-grid-map-msgs ros-melodic-grid-map-ros ros-melodic-vector-map-msgs 
sudo apt install ros-melodic-velodyne-pointcloud
sudo apt install ros-melodic-autoware-*
```



### Install

##### catkin build

```bash
git clone --recurse-submodules -j8 https://github.com/SS47816/auto_clean.git
cd auto_clean/
catkin build -j1
source devel_cb/setup.bash
```

***Notes:*** 

1. Use `catkin build -j1` for the first time to build the ros msgs

##### catkin_make

```bash
git clone --recurse-submodules -j8 https://github.com/SS47816/auto_clean.git
cd auto_clean/
catkin_make -j1
source devel/setup.bash
```



### Use

#### MulRan Dataset File Player

```
rosrun file_player file_player
```



#### SC-LeGO-LOAM

TODO



#### NDT Localizer

Copy and paste the pcd map into the `auto-clean/src/ndt_localizer/map/` folder:

```bash
cd auto-clean
cp map/kaist02.pcd src/ndt_localizer/map/

roslaunch ndt_localizer ndt_localizer.launch 
```

***Notes:*** 

1. For VLP-16, set the `leaf size` to be `2.0` [m]
2. 



#### Lanelet2 Router

1. Launch the nodes:

   ```bash
   roslaunch src/ad_with_lanelet2/run_map_simulator.launch
   ```

2. Select a starting point on the map using the `2D Pose Estimate` button in `Rviz`

3. Select a goal point on the map using the `2D Nav Goal` button in `Rviz`

***Notes:*** 

1. The starting and goal points selected doesn't have to be on the roads
2. They will be transformed to the nearest nodes



#### Multi-Lidar Calibration

1. 





---

## Backup Notes

# Autoware Packages

### Packages Used

* lidar_euclidean_cluster_detect
* points_preprocessor
* visualize_detected_objects
* lidar_imm_ukf_pda_track
* imm_ukf_pda_track

TODO:

* lidar_shape_estimation

### Dependencies

* Autoware

  * `common/autoware_build_flags/`

  * `messages/autoware_msgs/`

  * `common/vector_map_server/`
    
    * `common/vector_map/`
    
  * `visualization/detected_objects_visualizer/`

  * `messages/autoware_config_msgs/`

  * `common/autoware_health_checker/`

    * `common/ros_observer/`

  * `messages/autoware_system_msgs/`

  * `common/amathutils_lib/`

    

* ROS 

  ```bash
  sudo apt install ros-melodic-grid-map-cv ros-melodic-grid-map-msgs ros-melodic-grid-map-ros ros-melodic-vector-map-msgs 
  sudo apt install ros-melodic-velodyne-pointcloud
  sudo apt install ros-melodic-autoware-*
  ```


### Use

1. Set the params in `src/lidar_euclidean_cluster_detect/config/params.yaml`

2. Launch

   ```bash
   roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect_param.launch
   ```

3. 





## [GMapping]()

### Dependencies

* openslam_gmapping

  ```bash
  cd auto_clean/src
  git clone https://github.com/ros-perception/openslam_gmapping
  cd ..
  catkin_make
  ```

### Install

```bash
cd auto_clean/src
git clone https://github.com/ros-perception/slam_gmapping
cd ..
catkin_make
```

### Use

1. In the first terminal, start `roscore`

2. In the second terminal, play the rosbag

   ```bash
   rosbag play <bag_file_name>
   ```

3. In the third terminal, start `rviz` and select the topics

4. In the last terminal, 

   ```bash
   rosrun gmapping slam_gmapping scan:=base_scan
   ```




## [SC-LeGO-LOAM](https://github.com/SS47816/SC-LeGO-LOAM)

### Dependencies

* [GTSAM](https://github.com/borglab/gtsam)

  ```bash
  #!bash
  cd Downloads/
  wget -O ~/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
  cd ~ && unzip gtsam.zip -d ~
  cd ~/gtsam-4.0.0-alpha2/
  mkdir build && cd build
  cmake ..
  make -j 8
  sudo make install
  ```

  

### Install

Together with Mulran File Player

```bash
cd auto_clean/src
git clone https://github.com/AbangLZU/SC-LeGO-LOAM.git
git clone https://github.com/AbangLZU/file_player_mulran.git
cd ..
catkin_make
```

### Use

```bash
roslaunch file_player file_player.launch
roslaunch lego_loam run.launch
```





## [NDT](https://github.com/SS47816/ndt_localizer)

### Dependencies

*  [JSK ROS Packages](https://github.com/jsk-ros-pkg)

  ```bash
  sudo apt install ros-melodic-jsk-visualization
  ```

### Install

```bash
cd auto_clean/src
git clone https://github.com/AbangLZU/ndt_localizer.git
cd ..
catkin_make
```

### Use

1. Put the map in the `map/` folder and update the path in the `map_loader.launch`:

   ```xml
   <!--- MapLoader -->    
       <arg name="pcd_path"  default="$(find ndt_localizer)/map/kaist02.pcd"/>
   ```

2. Run 

   ```bash
   roslaunch file_player file_player.launch
   roslaunch ndt_localizer ndt_localizer.launch
   ```

   
