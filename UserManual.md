# ADLINK GMSL ROS2 Driver Usage

* Supported ROS version: ROS Foxy

# Installation

* Install ROS Foxy first.

  - See [Ubuntu install of ROS Foxy](https://docs.ros.org/en/foxy/Installation.html)

* Install the dependent packages.
  ```bash
  sudo apt install -y libgstreamer1.0-dev
  ```

* Install the debian package of ADLINK GMSL ROS2 Driver.

  ```bash
  sudo apt install ./ros-foxy-gmsl-ros2_<version>bionic_arm64.deb
  ```

* The ROS2 package will be installed under `/opt/ros/foxy`.

# Usage

* You can run ADLINK ROS2 driver after source the ROS Foxy.

  ```bash
  source /opt/ros/foxy/setup.bash
  ```

* Run the launch file.

  ```bash
  ros2 launch gmsl_ros2 <launch file name>
  ```

* There are some launch files can be run:
  - `gmsl_launch.py`: Publish the image received from GMSL.
  - `gmsl_composition_launch.py`: Use ROS 2 composition for zero copy.
  - `multi_camera_launch.py`: Two cameras example.
  - `multi_camera_composition_launch.py`: Two cameras example with ROS 2 composition for zero copy.

## gmsl_launch.py

Use the launch file to get raw image from GMSL.

```bash
ros2 launch gmsl_ros2 gmsl_launch.py

# Otherwise, open /dev/video2 and display the image in RViz
ros2 launch gmsl_ros2 gmsl_launch.py camera_dev:=/dev/video2 open_rviz:=true
```

Available parameters:

* open_rviz: Set it to true if you also want to open rviz. Default is false.
* camera_dev: The camera device you want to use. Default is `/dev/video0`.

Available topics:

* `/camera/image_raw`: Raw image from GMSL camera
* `/camera/camera_info`: The camera information for calibration

## gmsl_composition_launch.py

Use ROS 2 composition for both image publisher and subscriber can avoid memory copy, decrease CPU usage.

```bash
ros2 launch gmsl_ros2 gmsl_composition_launch.py

# Otherwise, open /dev/video2 and display the image in RViz
ros2 launch gmsl_ros2 gmsl_composition_launch.py camera_dev:=/dev/video2 open_rviz:=true
```

Available parameters:

* open_rviz: Set it to true if you also want to open rviz. Default is false.
* camera_dev: The camera device you want to use. Default is `/dev/video0`.

Available topics:

* `/camera/image_raw`: Raw image from GMSL camera
* `/camera/camera_info`: The camera information for calibration

## multi_camera_launch.py

Use this launch to run two GMSL cameras.

```bash
ros2 launch gmsl_ros2 multi_camera_launch.py

# Show image in rviz (will increase CPU usage)
ros2 launch gmsl_ros2 multi_camera_launch.py open_rviz:=true
```

Available parameters:

* open_rviz: Set it to true if you also want to open rviz. Default is false.

Available topics:

* `/camera0/image_raw`: Raw image from GMSL camera0
* `/camera0/camera_info`: The camera0 information for calibration
* `/camera1/image_raw`: Raw image from GMSL camera1
* `/camera1/camera_info`: The camera1 information for calibration

Note:

- Image rendering will increase the CPU usage, it's better not to show the image for saving the computing power.
- You can adjust the number of cameras and the namespace by modifying the launch file at `/opt/ros/foxy/share/gmsl_ros2/launch/multi_camera_launch.py`

## multi_camera_composition_launch.py

Use this launch to run two GMSL cameras with GMSL composable nodes.

```bash
ros2 launch gmsl_ros2 multi_camera_composition_launch.py

# Show image (will increase some CPU usage because of image rendering)
ros2 launch gmsl_ros2 multi_camera_composition_launch.py show_image:=true

# Show image in rviz (will increase more CPU usage because of memory copying to RViz)
ros2 launch gmsl_ros2 multi_camera_composition_launch.py open_rviz:=true
```

Available parameters:

* image_show: Set it to true to rendering the images by using nodelet image_view. Default is false.
* open_rviz: Set it to true if you also want to open rviz. Default is false.

Available topics:

* `/camera0/image_raw`: Raw image from GMSL camera0
* `/camera0/camera_info`: The camera0 information for calibration
* `/camera1/image_raw`: Raw image from GMSL camera1
* `/camera1/camera_info`: The camera1 information for calibration

Note:

- Image rendering will increase the CPU usage, it's better not to show the image for saving the computing power.
- You can adjust the number of cameras and the namespace by modifying the launch file at `/opt/ros/foxy/share/gmsl_ros2/launch/multi_camera_composition_launch.py`

# Uninstallation

  ```bash
  sudo apt remove ros-foxy-gmsl-ros2
  ```

# Zero-copy verification

```bash
# Terminal 1: publisher
ros2 launch gmsl_ros2 gmsl_composition_launch.py

# Terminal 2: Use iftop to verify zero-copy
sudo iftop -i lo

# Terminal 3: subscriber1
ros2 component load /camera_container image_view image_view::ImageViewNode -r image:=/camera/image_raw -n subscriber1 -e use_intra_process_comms:=true

# Terminal 4: subscriber2
ros2 component load /camera_container image_tools image_tools::ShowImage -r image:=/camera/image_raw -n subscriber2

# Terminal 5: subscriber3 w/o composable node, memory copy happens!
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw -r __name:=subscriber3
```