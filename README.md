# ADLINK GMSL ROS 2 Camera Driver

A ROS 2 Wrapper for GMSL camera

# Build

* Get the code

```bash
mkdir -p ~/gmsl_ros2_ws/src
cd gmsl_ros2_ws/src
git clone git@github.com:Adlink-ROS/gmsl_ros2.git
```

* Installing the dependent pkgs

```bash
cd ~/gmsl_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

* Build

```bash
cd ~/gmsl_ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run test
colcon test --event-handlers console_cohesion+
```

# Create deb file

```bash
# Ubuntu 20.04
sudo apt install dpkg-dev debhelper python3-bloom fakeroot

# Ubuntu 18.04
sudo apt install dpkg-dev debhelper python-bloom fakeroot

cd ~/gmsl_ros2_ws/src/gmsl_ros2
./create_deb.sh
```

You'll see the deb file in the parent folder.
The file name might be something like `ros-foxy-gmsl-ros2_0.1.0-0bionic_arm64.deb`

# Usage

Please see [UserManual.md](UserManual.md)
