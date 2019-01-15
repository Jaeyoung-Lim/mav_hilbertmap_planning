# mav_hilbertmap_planning
`mav_hilbertmap_planning` is a motion planning framework to safely navigate in the unknown environment using Hilbert Maps

## Prelimiaries
The prelimiary work on this project was done in matlab at the [unknown_mavplanning_matlab](https://github.com/Jaeyoung-Lim/unknown_mavplanning_matlab) repo.

## Dependencies
- px4 avoidance
- mav_voxblox_planning

## Running the code
The code can be launched by a launchfile using the following command.
```
roslaunch hilbert_mapper hilbert_mapper.launch
```
This will launch the `hilbert_mapper` node and the `local_planner` from the [PX4/Avoidance](https://github.com/PX4/avoidance) repo, which will pipe the pointcloud data from a intel-realsense into the hilbert mapper.
This is a test setup to verify that the pipeline is running. 