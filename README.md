# rbvogui_common

<a href="url"><img src="https://www.roscomponents.com/1206-big_default_2x/rb-vogui.jpg" height="275" width="275" ></a> 

Common packages of the RB-Vogui: URDF description of the RB-Vogui, platform messages and other files for simulation.

## packages

### rbvogui_control

This package contains the launch and configuration files to spawn the joint controllers (omni_drive_controller or ackermann_drive_controller) with the ROS controller_manager. 

### rbvogui_description

The urdf, meshes, and other elements needed in the description are contained here.

### rbvogui_localization

This package contains all the configuration files needed to execute the amcl algorithm serving a map or you can use the gmapping to make your own map of a world and test the amcl with it! 

### rbvogui_localization
This packages contain the files needed to run move_base or simple goals with move.
