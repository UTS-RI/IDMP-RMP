# IDMP-RMP

This is code for HRI25 paper "Enabling Safe, Active and Interactive Human-Robot
Collaboration via Smooth Distance Fields". IDMP ROS package contains the Gaussian Process distance field generation and rmp2_ros package contains the reactive planning which utilises IDMP. See each package for more details and instructions.

## Installation

See each package for dependencies and requirements before running the following.

```
mkdir -p idmp_ws/src
cd idmp_ws/src
git clone https://github.com/UTS-RI/IDMP-RMP
cd ..
catkin build
source devel/setup.bash
```

## Citation
If you use this source code, please cite the below article,

```
@inproceedings{10.5555/3721488.3721544,
author = {Ali, Usama and Sukkar, Fouad and Mueller, Adrian and Wu, Lan and Le Gentil, Cedric and Kaupp, Tobias and Vidal Calleja, Teresa},
title = {Enabling Safe, Active and Interactive Human-Robot Collaboration via Smooth Distance Fields},
year = {2025},
booktitle = {Proceedings of the 2025 ACM/IEEE International Conference on Human-Robot Interaction},
pages = {439–446},
numpages = {8},
location = {Melbourne, Australia},
}
```
