# Build & run package
* `git clone https://kcwian@bitbucket.org/raw-gnss/raw_gnss_rtklib.git`
* `catkin build raw_gnss_rtklib`
* `source ../devel/setup.bash`
* `roslaunch raw_gnss_rtklib raw_gnss_rtklib.launch`

By default RTKLIB output file is written to **./raw_gnss_rtklib/dataset/gps_solution_TST/results.pos**

# Evaluation
[evaluation](https://bitbucket.org/raw-gnss/raw_gnss_rtklib/src/master/evaluation/)
# References
* [RTKLIB](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)
* [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)
