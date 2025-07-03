# mvp_gui_2
ros2 version of mvp_gui

## Dependencies
**Python Packages**
- [numpy](https://numpy.org/install/)
    ```
    pip install numpy
    ```
- [pyyaml](https://pypi.org/project/PyYAML/)
    ```
    pip install PyYAML
    ```
- [python-socketio](https://python-socketio.readthedocs.io/en/stable/client.html#installation)
    ```
    pip install python-socketio
    ```
- [Flask](https://flask.palletsprojects.com/en/3.0.x/)
    ```
    pip install Flask
    ```
- [Flask-SQLAlchemy](https://flask-sqlalchemy.palletsprojects.com/en/3.1.x/quickstart/#installation)
    ```
    pip install Flask-SQLAlchemy
    ```
- [Flask WTF](https://flask-wtf.readthedocs.io/en/1.2.x/)
    ```
    pip install Flask-WTF
    ```
- [Flask-SocketIO](https://flask-socketio.readthedocs.io/en/latest/intro.html#installation)
    ```
    pip install Flask-SocketIO
    ```

**ROS Libraries**
- [geographic-msgs](https://docs.ros2.org/foxy/api/geometry_msgs/index-msg.html)
    ```
    sudo apt install ros-${ROS_DISTRO}-geographic-msgs
    ```
- [mvp_msgs](https://github.com/uri-ocean-robotics/mvp_msgs)
    ```
    git clone https://github.com/uri-ocean-robotics/mvp_msgs.git
    ```

## Offline map
- To obtain the offline maps stored in the submodule repository
    ```
    git submodule update --init --recursive
    ```
