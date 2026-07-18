#! /bin/bash
PROGRAM_PATH="$HOME/GitFiles/OpenSource/combination_slam"

# #plane_localization定位器
# gnome-terminal --tab -- bash -c "source $HOMEPROGRAM_PATH/setup.bash; roslaunch plane_localization start_construct.launch; exec bash"

#context_localization定位器
gnome-terminal --tab -- bash -c "source $HOMEPROGRAM_PATH/devel/setup.bash; roslaunch context_localization start_construct.launch; exec bash"