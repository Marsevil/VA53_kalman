# Description

This source code is for scholar practical. The goal is to compare different Kalman filter model in order to track a moving object in a video.

# How to use

## Build the project

### Dependencies

- OpenCV (Have to be installed on the system)
- Boost (Installed via conan)

### Instructions

- Run `cmake -B <path to build directory>` this will create a make project and run `conan install` automatically
- Run `cmake --build <path to build directory>`
- Run `<path to build directory>/bin/kalman_filter -h`
	> Help text is not provided yet

### Usage

First argument is the output csv file containing data about the trajectory (measured position, predicted position and corrected position).
The `--model-type` option can be used in order to modify the kalman model type

**Ex :** `kalman_filter data.csv --model-type=speed`

# To do

- [ ] Help message for `-h` or `--help`.
- [ ] Version message for `-v` or `--version`.
- [ ] Debug & fix passing video path in arguments.
- [ ] No output or output in stdout if no output path are provided.
- [ ] Expose options to setup Kalman covariance matrix.
- [ ] Draw on a mask instead of redrawing trajectories on each frame.

