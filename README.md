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

The * denotes dependencies which should be installed by following the install instructions on the respective webpages.

**Note:** Other versions of ROS may work, but this code has only been tested thoroughly on Indigo.

Use the following command to install dependencies:

```bash
$ sudo apt-get install g++ cmake libpopt-dev cimg-dev
```

## Compiling

### 1. ROS Environment Setup

`cd` to the directory where you want to install Human-in-the-Loop SLAM, and clone the repository.

```bash
$ git clone https://github.com/umass-amrl/hitl-slam
```

For compiling the ROS wrapper, `rosbuild` is used. `rosbuild` requires the path of the ROS wrapper to be added to 
the `ROS_PACKAGE_PATH` environment variable. To do this, add the following line in your `.bashrc` file. 

```bash
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/PATH/hitl-slam/HitL-SLAM:/PATH/hitl-slam/vector_slam_msgs
```

Replace `PATH` with the actual path where you have cloned the repository. 

### 2. Compiling HiTL-SLAM Source

To compile the source code, run the following commands.

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
$ make
```

## Using Human-in-the-Loop SLAM on Example Data

### 1. Download Datasets

Example data collected at University of Massachusetts Amherst, and used in the corresponding paper can be found 
[here](https://greyhound.cs.umass.edu/owncloud/index.php/apps/files/?dir=/laser_datasets/HitL_datasets/). In particular,
some of the following examples will use data from the file `2016-02-16-16-01-46.bag.stfs.covars` which can be found in the 
`Figure8` subdirectory within `/HitL_datasets`. Once downloaded, move the files into a directory of your choice, for example: 
`hitl-slam/HitL-SLAM/exampledata/`.

To execute HitL-SLAM, please see the section titled **Running Human-in-the-Loop SLAM**.

## Using Human-in-the-Loop SLAM on Standard Data or Your Own Data

### 1. Download Datasets

HitL-SLAM can be used on other datasets as well, as long as they are 2D, and based on depth scans or depth images. Many well-known datasets of this nature can be found [here](http://cres.usc.edu/radishrepository/view-all.php). After downloading a dataset or generating some data yourself, it needs to be put into the right format.

### 2. Convert Data to Homogenous Format

<!--- Data conversion tools for common formats coming soon --->

The HitL-SLAM I/O tools included in this source are designed to read from a very particular type of file. The file HitL-SLAM expects contains a string, the `map name`, on the first line, a float or double, the `timestamp`, on the second line, and then an arbitrary number of lines following, each with the identical format:

```
Pose_x, Pose_y, Pose_th, Obs_x, Obs_y, Normal_x, Normal_y, Covar_xx, Covar_xy, Covar_xth, Covar_yx, Covar_yy, Covar_yth, Covar_thx, Covar_thy, Covar_thth
```

Thus, a trivial example of an input file might look like

```
StarterMap
1455656519.379815
-0.0000,-0.0851,0.0967, -0.2685,-1.2530, -0.0132,0.9999, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000
-0.0000,-0.0851,0.0967, -0.2627,-1.2529, -0.0913,0.9958, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000, 0.00000
.
.
.
38.2454,-4.9589,-3.0867, 37.3000,-3.9542, 0.2121,-0.9772, 0.000703, 0.000008, 0.000009, 0.000008, 0.000714, -0.000150, 0.000009, -0.000150, 0.000117
38.2454,-4.9589,-3.0867, 37.2950,-3.9569, 0.3310,-0.9436, 0.000703, 0.000008, 0.000009, 0.000008, 0.000714, -0.000150, 0.000009, -0.000150, 0.000117
```

To execute HitL-SLAM, please see the section titled **Running Human-in-the-Loop SLAM**.

## Running Human-in-the-Loop SLAM

### 1. Starting ROS

By default, Human-in-the-Loop SLAM and the accompanying GUI are set up to run as ROS nodes. So, before proceeding, make sure there is a
`roscore` process running. Next, open two new terminals; one will be used to launch the GUI, and the other will used to run the HitL-SLAM node.







### 2. Command Line Arguments and Options

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

This software is released under the [GNU LGPL License](LICENSE).

