SESSION=floatsam_bringup

# Lidingo & Labbet
UTM_ZONE=34
UTM_BAND=V

# Kristineberg
#UTM_ZONE=32
#UTM_BAND=V

# Rest of Sweden
# UTM_ZONE=33
# UTM_BAND=V

# IP of SAM
FLOATSAM_IP=192.168.2.195

SSS_SAVE_PATH=/xavier_ssd/sidescan

# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux new-window -t $SESSION:1 -n 'gui'
tmux new-window -t $SESSION:2 -n 'core'
tmux new-window -t $SESSION:3 -n 'dr'
tmux new-window -t $SESSION:4 -n 'control'
tmux new-window -t $SESSION:5 -n 'mission'
#tmux new-window -t $SESSION:7 -n 'gps_dr'

tmux new-window -t $SESSION:6 -n 'rosbag'
#tmux new-window -t $SESSION:6 -n 'sam_monitor'
tmux new-window -t $SESSION:7 -n 'payloads'
# tmux new-window -t $SESSION:8 -n 'camera'
#tmux new-window -t $SESSION:8 -n 'rosbag'
#tmux new-window -t $SESSION:9 -n 'payloads'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

# start the gui and new_gui in one launch file 

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch floatsam_drivers floatsam_gui.launch rosbridge_ip:=$FLOATSAM_IP namespace:=floatsam --wait" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch floatsam_drivers floatsam_core.launch namespace:=floatsam  utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch floatsam_dead_reckoning floatsam_dr.launch namespace:=floatsam utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# tmux select-window -t $SESSION:4
tmux send-keys "mon launch floatsam_basic_controllers floatsam_control.launch namespace:=floatsam --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# tmux select-window -t $SESSION:5
tmux send-keys "mon launch floatsam_drivers floatsam_mission.launch namespace:=floatsam --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# tmux select-window -t $SESSION:6
tmux send-keys "mon launch floatsam_drivers floatsam_rosbag.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:7
tmux send-keys "mon launch floatsam_drivers floatsam_payloads.launch namespace:=floatsam --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
