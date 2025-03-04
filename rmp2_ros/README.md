# RMP2 for IDMP


Code adopted from https://github.com/UWRobotLearning/rmp2.git for HRI25 paper "Enabling Safe, Active and Interactive Human-Robot
Collaboration via Smooth Distance Fields"

### Dependencies
```
pip install -r requirements.txt

```

### Configuration

The RMP node subscribes to two topics:
    - /joint_states of the type JointStates where the robot driver should publish the state of its joints
    - /ur_hardware_interface/tcp_pose of the type TransformStamped where the robot driver should publish its tcp pose
Then it will publish joint velocities. As we are using a custom driver with a custom service, the service and its call needs to be adjusted in rmp2/rmp2/envs/ur5e_real.py for the used driver. 

### Running RMPs with IDMP

Run IDMP with your desired configuration

Run your robot driver. Make sure the URDF of the driver matches with the URDF used by rmps

```
rosrun rmp2_ros node.py
```

The following creates an interactive goal pose marker in rviz:
```
rosrun rmp2_ros goal_node.py
```

The following moves the arm between specified poses (warning: ensure poses in script are free of collision):
```
rosrun rmp2_ros pick_and_place.py
```

### Citation
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
