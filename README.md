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
- [popt](http://www.freshmeat.sourceforge.net/projects/popt)
- [CImg](http://www.cimg.eu/)
- [OpenMP](http://www.openmp.org/)
- [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)*
- [Ceres Solver](http://www.ceres-solver.org/)*

* Denotes dependencies which should be installed by following the install instructions on the respective webpages.

**Note:** Other versions of ROS may work, but this code has only been tested thoroughly on Indigo.

Use the following command to install dependencies:

```bash
$ sudo apt-get install g++ cmake libpopt-dev cimg-dev
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
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PATH/hitl-slam/HitL-SLAM:/PATH/hitl-slam/vector_slam_msgs
```

Replace `PATH` with the actual path where you have cloned the repository. To compile the source code, run the following commands.

```bash
$ cd ~/PATH_TO/hitl-slam/vector_slam_msgs
$ mkdir build
$ cd build
$ cmake ..
$ make
$ cd ~/PATH_TO/hitl-slam/HitL-SLAM/
$ mkdir build
$ cd build
$ cmake ..
$ cd ../
$ make
```













## Using Human-in-the-Loop SLAM on Example Data

TODO: find out where / how to host AMRL / LGRC data

### 1. Download Datasets

Example data collected at University of MAssachusetts Amherst, and used in the corresponding paper can be found 
[here](TODO). Once downloaded, move the files into a directory of your choice, for example: 
`hitl-slam/HitL-SLAM/exampledata/`.

To execute HitL-SLAM, please see the section titled **Running Human-in-the-Loop SLAM**.

## Using Human-in-the-Loop SLAM on Standard Data

### 1. Download Datasets

TODO: add links to some standard data

### 2. Converting Data to Homogenous Format

TODO: add instructions for using data conversion tools

## Using Human-in-the-Loop SLAM on Your Own Data

### 1. Convert Data to Homogenous Format

TODO: add instructions for using data conversion tools

## Running Human-in-the-Loop SLAM

### 1. Starting ROS

By default, Human-in-the-Loop SLAM and the accompanying GUI are set up to run as ROS nodes. So, before proceeding, make sure there is a
`roscore` process running. Next, open two new terminals; one will be used to launch the GUI, and the other will used to run the HitL-SLAM node.

### 2. Command Line Arguments and Options

TODO: finish
After compilation, the `HitL_SLAM` and `localization_gui` executables are stored in the `bin/` directory. 


### 3. Logging and Replaying

TODO: fill out

### 4. Saving Output Data

TODO: finish

### 5. Example Usage

TODO: finish and add pictures


|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/astar7-vis.jpg)       | ![](dumps/astar7-path.jpg)|


|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/rrt73-vis.jpg)        | ![](dumps/rrt73-path.jpg) |

## License

TODO: double check this
This software is released under the [MIT license](LICENSE).

