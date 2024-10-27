# FAST-LIO-Localization-SC-QN
+ This repository is a map-based localization implementation combining [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) as an odometry with [Quatro](https://quatro-plusplus.github.io/) and [Nano-GICP module](https://github.com/engcang/nano_gicp) as a map matching method, and with [ScanContext](https://github.com/gisbi-kim/scancontext_tro) as a loop candidate detection method
    + [Quatro](https://quatro-plusplus.github.io/) - fast, accurate and robust global registration which provides great initial guess of transform
    + [Quatro module](https://github.com/engcang/quatro) - `Quatro` as a module, can be easily used in other packages
    + [Nano-GICP module](https://github.com/engcang/nano_gicp) - fast ICP combining [FastGICP](https://github.com/SMRT-AIST/fast_gicp) + [NanoFLANN](https://github.com/jlblancoc/nanoflann)
    + [ScanContext](https://github.com/gisbi-kim/scancontext_tro) - a global descriptor for LiDAR point cloud, here it is used as loop candidate pair detection
+ Note: similar repositories already exist
    + [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): FAST-LIO2 + Open3D's ICP
    + [FAST-LIO-Localization-QN](https://github.com/engcang/FAST-LIO-Localization-QN): FAST-LIO2 + Quatro + Nano-GICP
+ Note2: main code is modularized and hence can be combined with any other LIO / LO
+ Note3: this repo is to apply `Quatro` in localization. `Quatro` can be worked in scan-to-scan matching or submap-to-submap matching but not scan-to-submap, i.e., ***the numbers of pointclouds to be matched should be similar.***
+ Note4: **saved map file** is needed. The map should be in `.bag` format. This `.bag` files can be built with [FAST-LIO-SAM-QN](https://github.com/engcang/FAST-LIO-SAM-QN) and [FAST-LIO-SAM](https://github.com/engcang/FAST-LIO-SAM)

## Video clip - https://youtu.be/MQ8XxRY472Y

<br>

## Main difference between FAST-LIO-Localization-QN and FAST-LIO-Localization-SC-QN
+ FAST-LIO-Localization-QN sets loop candidate pair as (current keyframe, the closest and old enough keyframe saved in the map data)
+ FAST-LIO-Localization-SC-QN gets loop candidate pair from ScanContext

<br>

## Dependencies
+ `C++` >= 17, `OpenMP` >= 4.5, `CMake` >= 3.10.0, `Eigen` >= 3.2, `Boost` >= 1.54
+ `ROS`
+ [`Teaser++`](https://github.com/MIT-SPARK/TEASER-plusplus)
    ```shell
    git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
    cd TEASER-plusplus && mkdir build && cd build
    cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
    sudo make install -j16
    sudo ldconfig
    ```
+ `tbb` (is used for faster `Quatro`)
    ```shell
    sudo apt install libtbb-dev
    ```

## How to build
+ Get the code and then build the main code.
    ```shell
    cd ~/your_workspace/src
    git clone https://github.com/engcang/FAST-LIO-Localization-SC-QN --recursive

    cd ~/your_workspace
    # nano_gicp, quatro first
    catkin build nano_gicp -DCMAKE_BUILD_TYPE=Release
    # Note the option!
    catkin build quatro -DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON
    catkin build -DCMAKE_BUILD_TYPE=Release
    . devel/setup.bash
    ```

## How to run
+ Then run (change config files in third_party/`FAST_LIO`)
    ```shell
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=ouster
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=velodyne
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=livox
    ```
* In particular, we provide a preset launch option for specific datasets:
    ```shell
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=kitti
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=mulran
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=newer-college20
    ```

<br>

## Structure
+ odomPcdCallback
    + pub realtime pose in corrected frame
    + keyframe detection -> if keyframe, add to pose graph + save to keyframe queue
+ matchingTimerFunc
    + process a saved keyframe
        + detect map match -> if matched, correct TF
    + visualize all

<br>

## Memo
+ `Quatro` module fixed for empty matches
+ `Quatro` module is updated with `optimizedMatching` which limits the number of correspondences and increased the speed

<br>

## License
<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a>

This work is licensed under a [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](http://creativecommons.org/licenses/by-nc-sa/4.0/)
