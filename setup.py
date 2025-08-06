# setup.py
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mvp_gui_2'

def find_files_in_dir(dir_name, file_types):
    """
    Recursively find all files of specified types in a directory, and
    prepare them for the data_files entry in setup.
    This function creates a list of (destination_directory, [source_files])
    tuples.
    """
    file_list = []
    # Assumes the source directory is under the package's python module,
    # e.g., 'mvp_gui/templates'. We want to install it to
    # 'share/package_name/templates'.
    base_dir = dir_name.split('/')[0] # e.g., 'mvp_gui'

    for (path, directories, filenames) in os.walk(dir_name):
        # Filter filenames to include only the specified file types
        source_files = [
            os.path.join(path, f) for f in filenames
            if any(f.endswith(ft) for ft in file_types)
        ]

        if source_files:
            # Create the destination path. We want to strip the python module
            # part from the path. e.g., 'mvp_gui/templates/subdir' becomes
            # 'templates/subdir'.
            relative_path = os.path.relpath(path, base_dir)
            install_dir = os.path.join('share', package_name, relative_path)
            file_list.append((install_dir, source_files))
            
    return file_list

# Find all HTML, CSS, JS, and image files for installation
template_files = find_files_in_dir('mvp_gui/templates', ['.html'])
static_files = find_files_in_dir('mvp_gui/static', ['.css', '.js', '.png', '.jpg', '.svg', '.ico'])

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + template_files + static_files,
    install_requires=[
        'setuptools',
        'Flask',
        'Flask-SocketIO',
        'Flask-SQLAlchemy',
        'Flask-WTF',
        'simple-websocket',
        'python-socketio',
        'python-engineio',
        'numpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='A ROS2 package for the MVP GUI, integrating a Flask web server.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_node = mvp_gui.nodes.flask_node:main',
            'ros_interface_node = mvp_gui.nodes.ros_interface_node:main',
        ],
    },
)