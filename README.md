# rbsherpa_common
Common packages of the RB-Sherpa: URDF description of the RB-Sherpa, platform messages and other files for simulation.

<h2>rbsherpa_control</h2>

Configuration and launch files to load omni_drive_controller.

Install external dependencies:

- [joint_read_command_controller](https://github.com/RobotnikAutomation/joint_read_command_controller)

- [universal_robot](https://github.com/fmauch/universal_robot.git)


```bash
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
```

Also you need to install dependencies, you need to execute:

```bash
sudo dpkg -i libraries/ros-kinetic-robotnik-msgs_1.0.0-0xenial_amd64.deb
```
```bash
sudo dpkg -i libraries/ros-kinetic-rcomponent_1.1.0-0xenial_amd64.deb
```
```bash
sudo dpkg -i libraries/ros-kinetic-ackermann-drive-controller_0.0.0-0xenial_amd64.deb
```
```bash
sudo dpkg -i libraries/ros-kinetic-omni-drive-controller_0.0.0-0xenial_amd64.deb
```
```bash
sudo dpkg -i libraries/ros-kinetic-robotnik-twist2ackermann_0.0.0-0xenial_amd64.deb
```

<h2>rbsherpa_description</h2>

The urdf, meshes, and other elements needed in the description are contained here.
