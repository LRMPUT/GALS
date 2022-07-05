# GALS (GNSS-Augmented LiDAR SLAM)

## Installation
#### Requirements
- [ROS](https://wiki.ros.org/ROS/Installation)

#### Clone **GALS** package
* `$ git clone https://github.com/kcwian/GALS.git`

#### Clone and build [g2o](https://github.com/RainerKuemmerle/g2o)
* `$ cd GALS && mkdir EXTERNAL && git clone https://github.com/RainerKuemmerle/g2o.git` 
* `$ cd g2o && mkdir build && cd build`
* `$ cmake .. -DG2O_HAVE_OPENGL=ON -DCMAKE_SKIP_RPATH=ON`
* `$ make -j4`

#### Build **GALS** package
* `catkin build gals`

#### Modify **gals.launch** file
* Set path of rover's GNSS observation data
  ```xml
    <arg name="roverMeasureFile" default="" />
  ```
* Set paths to GNSS navigation data
  ```xml
    <param name="navigationFile{1..5}" type="string" value="" />
     ```
* Set slamOdometryPath 
  ```xml
    <arg name="slamOdometryPath" default="" />
  ```

#### Run package
* `source ../devel/setup.bash`
* `roslaunch gals gals.launch`

# References
* [RTKLIB](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)
* [g2o](https://github.com/RainerKuemmerle/g2o)


# Licence
