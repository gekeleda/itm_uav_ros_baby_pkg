SETUP_FILE=devel/setup.bash

if [ -f "$SETUP_FILE" ]; then
  source devel/setup.bash
else
  source /opt/ros/melodic/setup.bash
fi

# setup Gazebo env and update package path
BUILD_DIR=~/Projektarbeit/PX4-Autopilot/build/px4_sitl_default
SRC_DIR=~Projektarbeit/PX4-Autopilot
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo
