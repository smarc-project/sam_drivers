SESSION=sam_bringup
# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'sam_core'
tmux new-window -t $SESSION:2 -n 'sam_dr'
tmux new-window -t $SESSION:3 -n 'sam_controllers'
tmux new-window -t $SESSION:4 -n 'sam_monitor'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys "mon launch sam_drivers sam_core.launch"

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_dead_reckoning sam_stim_dr.launch"

tmux select-window -t $SESSION:3
tmux send-keys "mon launch sam_drivers sam_controllers.launch"

tmux select-window -t $SESSION:4
tmux send-keys "mon launch sam_drivers sam_monitor.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
