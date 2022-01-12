# gmsl_ros2
A ROS 2 Wrapper for GMSL camera

## Zero-copy test

```bash
# Terminal 1
ros2 launch gscam2 gmsl_composition_launch.py

# Terminal 2
ros2 component load /camera_container image_tools image_tools::ShowImage -r image:=/camera/image_raw -n receiver1

# Terminal 3
ros2 component load /camera_container image_view image_view::ImageViewNode -r image:=/camera/image_raw -n receiver2

# Terminal 4
sudo iftop -i lo
```

## gtest

```bash
colcon build
colcon test --event-handlers console_cohesion+
```