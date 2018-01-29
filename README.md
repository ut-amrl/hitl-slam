# Human-in-the-Loop SLAM

This repository contains a C++ implementation of Human-in-the-Loop SLAM, for use in post-hoc metric map repair and correction. A ROS 
wrapper is provided, which handles communication between GUI and backend. The source code is designed to be readable and extensible, and 
can be easily modified to fit the needs of a particular project.

- Author: [Samer Nashed](TODO)

## Publication

If you use this software in an academic work or find it relevant to your research, kindly cite:

```
@article{nashed2017human,
  title={Human-in-the-Loop SLAM},
  author={Nashed, Samer B and Biswas, Joydeep},
  journal={arXiv preprint arXiv:1711.08566},
  year={2017}
}
```

Link to paper: [https://arxiv.org/pdf/1711.08566.pdf](https://arxiv.org/pdf/1711.08566.pdf)

## Dependencies

- A C++ compiler (*e.g.*, [GCC](http://gcc.gnu.org/))
- [cmake](http://www.cmake.org/cmake/resources/software.html)
- [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
- [Ceres Solver](http://www.ceres-solver.org/)
- [CImg](http://www.cimg.eu/)
- TODO: finish this list  

**Note:** Other versions of ROS may work, but this code has only been tested thoroughly on Indigo.

Use the following command to install dependencies:

```bash
$ sudo apt-get install cimg-dev TODO: finish
```


## Compiling

### 1. Building HiTL-SLAM

`cd` to the directory where you want to install Human-in-the-Loop SLAM, and clone the repository.

```bash
$ git clone https://github.com/umass-amrl/hitl-slam
```

For compiling the ROS wrapper, `rosbuild` is used. `rosbuild` requires the path of the ROS wrapper to be added to 
the `ROS_PACKAGE_PATH` environment variable. To do this, add the following line in your `.bashrc` file. 


```bash
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PATH/hitl-slam:/PATH/hitl-slam/vector_slam_msgs
```

Replace `PATH` with the actual path where you have cloned the repository. To compile the source code, run the script `TODO.sh` with the following commands.

```bash
$ cd TODO
```

## Using Human-in-the-Loop SLAM on Example Data

TODO: find out where / how to host AMRL / LGRC data

### 1. Download Datasets

Example data collected at University of MAssachusetts Amherst, and used in the corresponding paper can be found 
[here](TODO). Once downloaded, move the files into the `hitl-slam/exampledata/` directory.

To

## Using Human-in-the-Loop SLAM on Standard Data

### 1. Download Datasets

TODO: add links to some standard data

### 2. Converting Data to Homogenous Format

TODO: add instructions for using data conversion tools

## Using Human-in-the-Loop SLAM on Your Own Data

### 1. Convert Data to Homogenous Format

TODO: add instructions for using data conversion tools

## Running Human-in-the-Loop SLAM

### Starting ROS

By default, Human-in-the-Loop SLAM and the accompanying GUI are set up to run as ROS nodes. So, before proceeding, make sure there is a
`roscore` process running.

### Command Line Arguments and Options


After compilation, the `HitL_SLAM` and `localization_gui` executables are stored in the `bin/` directory. 







```bash
$ ./bin/jpp -n [number of pairs] -d [path/to/directory] -c [path/to/stereo/calibration/file] -j [path/to/jpp/config/file] -o [output_mode]
```

**Note:** stereo image pairs inside the directory must be named like this: `left1.jpg`, `left2.jpg`, ... , `right1.jpg`, `right2.jpg`, ...

For the example datasets, calibration files are stored in the `calibration/` folder and JPP configurations are stored in the `cfg/` folder. JPP operates on 
3 output modes (set by the `-o` flag) as of now: `astar`, `rrt`, and `debug` mode. Set the flag `-v 1` for generating visualizations.

```bash
Usage: jpp [OPTION...]
  -n, --num_imgs=NUM            Number of images to be processed
  -d, --img_dir=STR             Directory containing image pairs (set if n > 0)
  -l, --left_img=STR            Left image file name
  -r, --right_img=STR           Right image file name
  -c, --calib_file=STR          Stereo calibration file name
  -j, --jpp_config_file=STR     JPP config file name
  -o, --output=STR              Output - astar, rrt, debug
  -v, --visualize=NUM           Set v=1 for displaying visualizations
  -w, --write_files=NUM         Set w=1 for writing visualizations to files
```

For example, running JPP on the KITTI dataset in `astar` mode:

```bash
$ ./bin/jpp -n 33 -d KITTI/ -c calibration/kitti_2011_09_26.yml -j cfg/kitti.cfg -o astar -v 1
```

|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/astar7-vis.jpg)       | ![](dumps/astar7-path.jpg)|

Running JPP on the AMRL dataset in `rrt` mode:

```bash
$ ./bin/jpp -n 158 -d AMRL/ -c calibration/amrl_jackal_webcam_stereo.yml -j cfg/amrl.cfg -o rrt -v 1
```

|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/rrt73-vis.jpg)        | ![](dumps/rrt73-path.jpg) |



### Saving Output Data

TODO: 

## License

TODO: double check this
This software is released under the [MIT license](LICENSE).

