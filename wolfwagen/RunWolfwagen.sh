#!/bin/bash
session="Wolfwagen"
RESULTONE="no server running on /tmp/tmux-1000/default"
RESULTTWO="Can't find session: $session"
RESULTBASE=`tmux has-session -t $session 2>&1`
# diff <(echo "-w") <(echo "$RESULT" ) <(echo "$RESULTONE")
# cmp <(echo "$RESULT") <(echo "$RESULTONE")
echo "$RESULTBASE"

if [ "$RESULTBASE" == "Can't find session: $session" ] || [ "$RESULTBASE" == "no server running on /tmp/tmux-1000/default" ] || [ "$RESULTBASE" == "error connecting to /tmp//tmux-1000/default (No such file or directory)" ]
then
	echo "Running Wolfwagen"
	tmux new-session -d -s $session
	tmux rename-window 'Main'
	tmux select-window -t $session:0
	tmux split-window -h 'exec ros2 launch zed_wrapper zed2i.launch.py'
	tmux split-window -v 'exec ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0'
	tmux split-window -h 'exec python3 pwm_genV3.py'
	tmux split-window -v 'exec python3 LaneDetectionV6.2.py'
	tmux split-window -h 'exec python3 xbox_controller.py'
	tmux split-window -v 'exec cd stop_sign_detection'
	tmux send-keys './stop_sign_detect_node.py' C-m
	tmux split-window -h 'exec ros2 launch sllidar_ros2  sllidar_s2_launch.py'
	tmux split-window -v 'exec python3 obstacle_detector.py'
	tmux split-window -h 'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
else
	echo "Wolfwagen is already running"
fi

