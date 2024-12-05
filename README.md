# Arena4-IsaacSim.

## Tasks.
Working on:
 - [x] IsaacSim launch file. (need more update: An)
 - [x] Service add usd file to environment. (An)
 - [x] Service convert urdf to usd. (Giang)
 - [ ] Service convert sdf to usd. (pending)
 - [x] Publish robot joint states. (Lam)
 - [ ] Publish robot sensors. (current: lidar, camera rgb, camera depth: Kien)
 - [ ] Convert all urdf robot files to usd files. (on working: Kien)
 - [x] Update model states. (Duc Anh)
 - [x] Connect velocity commands from navigation stack to control commands. (An)
 - [ ] Service update model states. (Duc Anh)
 - [x] Service spawn with file content. (Kien)
 - [ ] Robot state: parse urdf, recognize sensors (maybe change to usd files: Kien)
 - [x] Service move and set orientation (orientation in quaternion and euler: Duc Anh)
 - [ ] Validate sensor in rviz (Kien)
 - [ ] Write isaac_simulator.py (An) 
       
## How to run.

You need to install ros2 before run this scripts 
```
https://docs.ros.org/en/humble/index.html
```
clone the project.
```
https://github.com/Arena-Rosnav/arena-isaac.git
cd arena-isaac
```

create the environment and install dependencies.
```
./create_env
```

run colcon build for start working.
```
./colcon_build
```

*NOTE: the installation just long, do not interupt.

*NOTE: you need to source the init.bash before run.
