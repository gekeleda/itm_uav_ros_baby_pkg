# itm_uav_ros_baby_pkg

A baby package for beginner to build his own UAV package.

## Some required packages

* [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) official package for PX4 firmware for Pixhawk flight controller.
* Customized software-in-the-loop model from ITM [sitl_gazebo](https://github.com/tomcattigerkkk/PX4-SITL_gazebo). Please only copy the files that are required by your simulation.
* Some special customized message definitions (including msg/srv/action) from [itm_ros_comm](https://github.com/tomcattigerkkk/itm_ros_comm).
* ACADOS should be installed following the instructions from official website [ACADOS](https://github.com/acados/acados).

## Environment setup

create a script file in the work space, such as ``ros.sh`` with following codes

``` bash
conda activate xxx # if you have some special conda environment
# note that, if you use another shell, please change zsh to your shell type accordingly
SETUP_FILE=devel/setup.zsh

if [ -f "$SETUP_FILE" ]; then
  source devel/setup.zsh
else
  source /opt/ros/melodic/setup.zsh
fi

# setup Gazebo env and update package path
BUILD_DIR={where you store PX4-Autopilot}/build/px4_sitl_default
SRC_DIR={where you store PX4-Autopilot}
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo
```

## To implement

This package provides a simple implementation of MPC-based controller in ROS for a UAV working with PX4.

Note that, one should implement the trajectory reference node, which indicates the desired trajectory for MPC controller.

## Process

1. At Terminal 1, go to the folder where you store PX4-Autopilot, then execute

    ``` bash
    no_sim=1 make px4_sitl_default gazebo
    ```

2. Then in the second Terminal, source the environment, [see above introduction](#Environment-setup), and launch the ``px4_sitl.launch`` file to call the simulation platform.

3. At the third and fourth Terminal, one can launch his trajectory generator and MPC controller, respectively.
