# MVP GUI 2

MVP GUI 2 is a comprehensive web-based graphical user interface designed for monitoring and controlling ROS 2 based autonomous vehicles, particularly underwater vehicles. This package provides real-time visualization of vehicle status and enables operators to control various aspects of the vehicle through a web interface.

## Usage

### Installation

1.  Clone the repository into your ROS 2 workspace's `src` directory.
2.  Install dependencies:
    - via `rosdep`:
        ```bash
        rosdep install --from-paths src --ignore-src -r -y
        ```
    - manually:
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

3.  Clone required dependency repositories (if not available via `rosdep`), e.g.:
    ```bash
    git clone https://github.com/uri-ocean-robotics/mvp_msgs.git
    ```
4.  Initialize submodules to get offline map data:
    ```bash
    cd /path/to/your/ros2_ws/src/mvp_gui_2
    git submodule update --init --recursive
    ```
5.  Build the workspace:
    ```bash
    colcon build
    ```

### Configuration

1.  Parameters can be set in `config/mvp_gui_params.yaml`
2.  Ensure topic and service names match your ROS 2 system.
3.  Build the workspace

### Running the Application

1.  Source your ROS 2 workspace:
    ```bash
    source /path/to/your/ros2_ws/install/setup.bash
    ```
2.  Launch the application using the ROS 2 launch file:
    ```bash
    ros2 launch mvp_gui_2 mvp_gui_2.launch.py
    ```
3.  Access the web interface in your browser, typically at `http://localhost:5001`.

### Web Interface Navigation

1.  **Systems**: System status and launch control.
2.  **Vehicle Status**: Real-time vehicle information.
3.  **Power Manager**: Power system control.
4.  **Mission**: Waypoint and mission management.
5.  **Map**: Geospatial visualization.

### Control Operations

1.  Change vehicle states using the helm state controls on the Map page.
2.  Enable/disable the controller as needed.
3.  Control individual power systems from the Power Manager page.
4.  Manage ROS launch files from the Systems page.
5.  Publish waypoints for navigation missions from the Map or Mission page.

## Additional Documentation
Detailed Overview of the pacakge can be found in [docs/mvp_gui_2_overview](docs/mvp_gui_2_overview.md)