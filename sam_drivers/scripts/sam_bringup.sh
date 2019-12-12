SESSION=sam_bringup
# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'gui'
tmux new-window -t $SESSION:2 -n 'core'
tmux new-window -t $SESSION:3 -n 'dvl'
tmux new-window -t $SESSION:4 -n 'dr'
tmux new-window -t $SESSION:5 -n 'static_ctrl'
tmux new-window -t $SESSION:6 -n 'dyn_ctrl'
tmux new-window -t $SESSION:7 -n 'gps_dr'
tmux new-window -t $SESSION:8 -n 'bt'
#tmux new-window -t $SESSION:6 -n 'sam_monitor'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "rosrun flexxros sam_controls.py" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_drivers sam_core.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch dvl_ros_driver dvl.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:4
tmux send-keys "mon launch sam_dead_reckoning sam_stim_dr.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:5
tmux send-keys "mon launch sam_basic_controllers static_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:6
tmux send-keys "mon launch sam_basic_controllers dynamic_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:7
tmux send-keys "mon launch sam_dead_reckoning sam_gps_dummy.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:8
tmux send-keys "mon launch sam_march sam_march.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

#tmux select-window -t $SESSION:8
#tmux send-keys "roslaunch sam_drivers sam_monitor.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
