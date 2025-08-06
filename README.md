# MVP GUI 2
### ROS 2 MVP GUI.
- MVP GUI 2 is a comprehensive web-based graphical user interface designed for monitoring and controlling ROS 2 based autonomous vehicles, particularly underwater vehicles. This package provides real-time visualization of vehicle status and enables operators to control various aspects of the vehicle through a web interface.

## Installation

### Dependencies

This package relies on several system libraries and ROS packages. They can be installed via `apt`.

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-tf-transformations \
    python3-flask \
    python3-flask-socketio \
    python3-flask-sqlalchemy \
    python3-flask-wtf \
    python3-socketio \
    python3-simple-websocket \
    python-engineio \
    python3-numpy \
    python3-pyyaml
```

### Offline map
To obtain the offline maps stored in the submodule repository [mvp_gui_offline_map](https://github.com/altyi/mvp_gui_offline_map/tree/4420bff8279bfa7e7c3ae994d818486b09b1d105).
```bash
git submodule update --init --recursive
```

## How to Run
### Configuration
Parameters can be set in `config/mvp_gui_params.yaml`

### Ros2 Launch
```bash
ros2 launch mvp_gui_2 mvp_gui_2.launch.py
```