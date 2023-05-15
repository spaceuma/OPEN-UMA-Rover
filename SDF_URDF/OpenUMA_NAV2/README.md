# Open UMA files for nav2_bringup

This files must be placed into `nav2_bringup` directory in your navigation2 ws. The folder's names that are used here match the names in `nav2_bringup`. 

This code is a modification of the basic .sdf and .urdf models of OPEN UMA robot so that they can work properly in nav2. It also includes the launch file and the simulation enviroment.

PLEASE NOTICE THAT:

1. `nav2_params` is configured to use `Fast Marching` and `Waypoint Navigation` algotihms.
2. You must specify where `worlds` directory is. For example: export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/<user_name>/nav2_ws/src/navigation2/nav2_bringup/worlds
3. This open uma model has more sensors that the basic version. This allows to test different localization algorithms.
4. `xacros` doesn't work properly en nav2.


