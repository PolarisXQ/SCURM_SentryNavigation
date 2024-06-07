# Analysis path with EVO

## Install

pip3 install evo

## Usage

ros2 bag record /Odometry

evo_traj bag2 ./rosbag2_2024_02_26-09_28_04/ /Odometry --save_as_tum

evo_rpe tum state_estimation_point_lio.tum Odometry.tum -v -p