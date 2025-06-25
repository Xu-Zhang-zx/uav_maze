# !/bin/bash
# tmux
cmd=$(which tmux)
session=fuel
winName=mission

if [-z $cmd ]; then
	echo "You need to install tmux."
	exit 1
fi

$cmd kill-session -t $session

$cmd has -t $session

if [ $? != 0 ]; then
    $cmd new -s $session -d -n $winName
    
    # 分割顶部窗格（占50%高度）
    $cmd splitw -v -p 50 -t $winName
    
    # 返回顶部窗格（pane 0）
    $cmd selectp -t 0
fi

$cmd selectp -t 0
$cmd send-keys "cd /home/xu/Code/uav_maze;source devel/setup.bash;roslaunch exploration_manager rviz.launch" C-m
$cmd selectp -t 1
$cmd send-keys "sleep 5;cd /home/xu/Code/uav_maze;source devel/setup.bash;roslaunch exploration_manager exploration_uav_maze.launch" C-m

$cmd att -t $session
