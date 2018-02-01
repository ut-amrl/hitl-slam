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
- [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)* or later
- [Ceres Solver](http://www.ceres-solver.org/)*

The * denotes dependencies which should be installed by following the install instructions on the respective webpages.

**Note:** Previous versions of ROS may work, but this code has only been tested thoroughly on Indigo and later distributions.

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
$ cd ~/PATH_TO/hitl-slam/vector_slam_msgs/
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

Thus, an example of an input file might look like

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

After compilation, the `HitL_SLAM` and `localization_gui` executables will show up in the `bin/` directory. To start the GUI, in a new terminal run the `localization_gui` executable.

```
$ cd ~/PATH_TO/hitl-slam/HitL-SLAM
$ ./bin/localization_gui
```

<!--- TODO: describe features / options of the gui --->

The `HitL_SLAM` executable can be run in a similar fashion, but also requires two command line arguments. The first is an options flag, which must take one of two values: `-P` or `-L`. The second is the path a file e.g. `exampledata/2016-02-16-16-01-46.bag.stfs.covars`. An example command to run Human-in-the-Loop SLAM is

```
$ cd ~/PATH_TO/hitl-slam/HitL-SLAM
$ ./bin/HitL_SLAM -P exampledata/2016-02-16-16-01-46.bag.stfs.covars
```

`-P` signals a start from scratch, while `-L` tells HitL-SLAM that it will be loading a previous session, stored in a log file. 

For a full example of all main HitL-SLAM features, see the **Example Usage** section.

### 3. Logging and Replaying

Humain-in-the-Loop sessions will automatically be logged. Logs are written when the node receives a CTRL-C signal. Logs can be loaded and replayed using the `-L` option in addition to the `-P` option. once loaded, pressing `l` while in the GUI will step through the log, one correction at a time. After the log has played back fully, the user can resume making corrections.

### 4. Entering Corrections

To make corrections, first make sure you are focused on the GUI window. Press the `p` key to enter 'provide correction mode'. Once in this mode hold down one or more of the correction type specifier keys, and select the two features you wish to relate. Once you are done, press the `p` key again to trigger the algorithm in full.

Correction Type Specifier Key Combinations:
```
CTRL = Colocation
SHIFT = Collinear
SHIFT + ALT = Perpendicular
SHIFT + CTRL = Parallel
```

### 5. Undoing a Correction

If you don't like the outcome of a given correction, you can undo it by pressing the `u` key while in the GUI. This reverts the pose and covariance estimates to their previous state, and removes the most recent set of human correction factors from the factor graph.

### 6. Saving Output Data

To save the pose estimates, press the `v` key while in the GUI. The pose estimates will be written to a default output file `hitl_results.txt`. To specify a file name to save to, use the `-V` command line option followed by the desired file name.

### 7. Example Usage

TODO: start up, make a correction, log it, quit, load a log, replay, make a second correction, undo it, do it again, save

<!---
|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/astar7-vis.jpg)       | ![](dumps/astar7-path.jpg)|
--->
<!---
|Confidence match visualizations | Path visualization        |
|:------------------------------:|:-------------------------:|
|![](dumps/rrt73-vis.jpg)        | ![](dumps/rrt73-path.jpg) |
--->

## License

This software is released under the [GNU LGPL License](LICENSE).
