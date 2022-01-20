# Build & run package
* `git clone https://kcwian@bitbucket.org/raw-gnss/raw_gnss_rtklib.git`
* `catkin build raw_gnss_rtklib`
* `source ../devel/setup.bash`
* `roslaunch raw_gnss_rtklib raw_gnss_rtklib.launch`

By default RTKLIB output file is written to **./raw_gnss_rtklib/dataset/gps_solution_TST/results.pos**

# Evaluation
* [Evaluation](https://bitbucket.org/raw-gnss/raw_gnss_rtklib/src/master/evaluation/)
# References
* [RTKLIB](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3)
* [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)

# Build g2o
- `git clone https://github.com/RainerKuemmerle/g2o.git`
- `cd g2o && mkdir build && cd build`
- `cmake .. -DG2O_HAVE_OPENGL=ON -DCMAKE_SKIP_RPATH=ON` (Options if something is not working)
- `make install`
- echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc

