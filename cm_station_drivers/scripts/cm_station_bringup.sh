SESSION=cm_station_bringup

# Lidingo & Labbet
# UTM_ZONE=34
# UTM_BAND=V

# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'uwcomms'
tmux new-window -t $SESSION:3 -n 'uwgps'


tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

# start the gui and new_gui in one launch file 

tmux select-window -t $SESSION:1
tmux send-keys "mon launch cm_station_drivers cm_station_core.launch namespace:=cm_station  --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch cm_station_drivers cm_station_uwcomms.launch namespace:=cm_station --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch cm_station_drivers cm_station_uwgps.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
