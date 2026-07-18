#! /bin/bash
PROGRAM_PATH="$HOME/GitFiles/OpenSource/combination_slam"

#fast_lio里程计
gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch fast_lio lio_livox.launch; exec bash"

#dlio里程计
#gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch direct_lidar_inertial_odometry dlio_odom.launch; exec bash"

#sam建图后端优化
sleep 5
gnome-terminal --tab -- bash -c "source $PROGRAM_PATH/devel/setup.bash; roslaunch sam_back_end run.launch; exec bash"