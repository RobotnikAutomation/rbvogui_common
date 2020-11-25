# rbsherpa_common

<a href="url"><img src="https://www.roscomponents.com/1206-big_default_2x/rb-sherpa.jpg" height="275" width="275" ></a> 

Common packages of the RB-Sherpa: URDF description of the RB-Sherpa, platform messages and other files for simulation.

## packages

### rbsherpa_control

This package contains the launch and configuration files to spawn the joint controllers (omni_drive_controller or ackermann_drive_controller) with the ROS controller_manager. 

### rbsherpa_description

The urdf, meshes, and other elements needed in the description are contained here.

### rbsherpa_localization

This package contains all the configuration files needed to execute the amcl algorithm serving a map or you can use the gmapping to make your own map of a world and test the amcl with it! 

### rbsherpa_localization
This packages contain the files needed to run move_base or simple goals with move.
