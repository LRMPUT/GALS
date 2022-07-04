# GALS (GNSS-Augmented LiDAR SLAM)

## Installation
#### Requirements
- [ROS](https://wiki.ros.org/ROS/Installation)
- [g2o](https://github.com/RainerKuemmerle/g2o)


#### Clone and build  package
* `git clone https://github.com/kcwian/GALS.git`
* `catkin build gals`

#### Modify **gals.launch** file
* Set path of rover's GNSS observation data
* Set paths to GNSS navigation data
* Set slamOdometryPath   

#### Run package
* `source ../devel/setup.bash`
* `roslaunch gals gals.launch`

# References
* [RTKLIB](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)
* [g2o](https://github.com/RainerKuemmerle/g2o)


# Licence
