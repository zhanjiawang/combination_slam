#! /bin/bash
PROGRAM_PATH="$HOME/GitFiles/OpenSource/combination_slam"

#fast_lio里程计
gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch fast_lio lio_livox.launch; exec bash"

# #dlio里程计
#gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch direct_lidar_inertial_odometry dlio_odom.launch; exec bash"

#plane_localization定位器
sleep 5
gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch plane_localization start_localization.launch; exec bash"

# #context_localization定位器
# sleep 5
# gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch context_localization start_localization.launch; exec bash"