#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

PROJECT_NAME=grasping

MAIN_DIR=~/"bag_files"

# following commands will be executed first, in each window
pre_input="export ATHAME_ENABLED=0; mkdir -p $MAIN_DIR/$PROJECT_NAME"

# define commands
# 'name' 'command'
input=(
  'Rosbag' 'waitForOffboard; ./record_brick.sh
'
  'Sensors' 'waitForRos; roslaunch mrs_general sensors.launch
'
  'Optflow' 'waitForRos; roslaunch mrs_optic_flow optic_flow.launch
'
  'Nimbro' 'waitForRos; roslaunch mrs_general nimbro.launch custom_config:=./custom_configs/nimbro.yaml
'
  'Detection' 'waitForRos; roslaunch brick_detection brick_detection.launch 
'
  'Estimation' 'waitForRos; roslaunch brick_estimation brick_estimation.launch custom_config:=./custom_configs/nimbro.yaml
'
  'Grasping' 'waitForRos; roslaunch brick_grasping brick_grasping.launch
'
  'Gripper' 'waitForRos; roslaunch mrs_gripper uav.launch
'
  'AutoStart' 'waitForRos; roslaunch automatic_start_mbzirc automatic_start_mbzirc.launch challenge:=wall
'
  'Status' 'waitForRos; roslaunch mrs_status status.launch
'
  'Control' 'waitForRos; roslaunch mrs_general core.launch config_odometry:=./custom_configs/odometry.yaml config_gain_manager:=./custom_configs/gain_manager.yaml config_uav_manager:=./custom_configs/uav_manager.yaml config_landoff_tracker:=./custom_configs/landoff_tracker.yaml config_constraint_manager:=./custom_configs/constraint_manager.yaml  config_mpc_controller:=./custom_configs/mpc_controller.yaml custom_config:=./custom_configs/nimbro.yaml config_mpc_tracker:=./custom_configs/mpc_tracker.yaml
'
  'Start' 'rosservice call /'"$UAV_NAME"'/brick_grasping/start 0'
  'CalibrateWall' 'rosservice call /'"$UAV_NAME"'/brick_detection/type 9'
  'GroundPlace' 'rosservice call /'"$UAV_NAME"'/brick_grasping/ground_place'
  'WallPlace' 'rosservice call /'"$UAV_NAME"'/brick_grasping/wall_place'
  'GripStatus' 'rostopic echo /'"$UAV_NAME"'/gripper/gripper_diagnostics
'
  'Grip' 'rosservice call /'"$UAV_NAME"'/gripper/grip'
  'Ungrip' 'rosservice call /'"$UAV_NAME"'/gripper/ungrip'
  'DebilLand' 'rosservice call /'"$UAV_NAME"'/control_manager/partial_land'
  'ChangeEstimator' 'waitForOdometry; rosservice call /'"$UAV_NAME"'/odometry/change_estimator_type_string GPS'
  'Land' 'rosservice call /'"$UAV_NAME"'/uav_manager/land'
  'LandHome' 'rosservice call /'"$UAV_NAME"'/uav_manager/land_home'
  'Show_odom' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/slow_odom
'
  'Show_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/odometry/diagnostics
'
  'Mav_diag' 'waitForRos; rostopic echo /'"$UAV_NAME"'/mavros_interface/diagnostics
'
  'KernelLog' 'tail -f /var/log/kern.log -n 100
'
  'roscore' 'roscore
'
  'KILL_ALL' 'dmesg; tmux kill-session -t '
)

init_window="Status"

###########################
### DO NOT MODIFY BELOW ###
###########################

SESSION_NAME=mav

FOUND=$( /usr/bin/tmux ls | grep mav )

if [ $? == "0" ]; then

  echo "The session already exists"
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= /usr/bin/tmux new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest"
rm "$MAIN_DIR/latest"
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  /usr/bin/tmux new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  /usr/bin/tmux pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  tmux send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

/usr/bin/tmux select-window -t $SESSION_NAME:$init_index

# /usr/bin/tmux -2 attach-session -t $SESSION_NAME
