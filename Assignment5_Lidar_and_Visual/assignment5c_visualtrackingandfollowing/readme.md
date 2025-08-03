# Visual Tracking
## Installation
1. Use ``catkin_create_pkg`` to make package called assignment5c_linefollower
2. Clone the repo to your package directory, either directly or using symbolic links
3. Run ``catkin_make`` in your termnial.
4. Be sure to have run or added the following commands to your ``.bashrc`` and source it
   * ``export TURTLEBOT3_MODEL=burger``
   *  ``source devel/setup.bash``
## Running the package
1. After installation, you can run ``roslaunch assignment5c_linefollower blob_tracker.launch ``
   *This should launch a Gazebo instance as well as the Python controller script
## Videos


https://github.com/BleeKelly/Assignment5_Lidar_and_Visual/assets/150833244/941bd1a0-51c0-4cd5-a45d-ee057aa19578

Terminal output of line follower


https://github.com/BleeKelly/Assignment5_Lidar_and_Visual/assets/150833244/12365a12-4db2-4320-859f-dda03a65094a

Gazebo output of line follower
