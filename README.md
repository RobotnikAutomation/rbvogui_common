# rbsherpa_common
Common packages of the RB-Sherpa: URDF description of the RB-Sherpa, platform messages and other files for simulation.

<a href="url"><img src="https://www.roscomponents.com/1206-big_default_2x/rb-sherpa.jpg" align="left" height="275" width="275" ></a> 
## installation

Install external dependencies:

- [universal_robot](https://github.com/fmauch/universal_robot.git) and switch to the calibration_devel branch typing the following command.

```bash
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
```

Also you need to install dependencies, you need to execute:

```bash
ros-melodic-ackermann-drive-controller_0.0.0-0bionic_amd64.deb
```
```bash
ros-melodic-omni-drive-controller_0.0.0-0bionic_amd64.deb
```
```bash
ros-melodic-robotnik-twist2ackermann_0.0.0-0bionic_amd64.deb
```

## packages

### rbsherpa_control

This package contains the launch and configuration files to spawn the joint controllers (omni_drive_controller or ackermann_drive_controller) with the ROS controller_manager. 

### rbsherpa_description

The urdf, meshes, and other elements needed in the description are contained here.

### rbsherpa_navigation

This package contains all the configuration files needed execute the AMCL algorithms in simulation. 

### rbsherpa_localization

