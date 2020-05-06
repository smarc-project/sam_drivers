SESSION=sam_bringup

# Lidingo
UTM_ZONE=34
UTM_BAND=V

# KTH
# UTM_ZONE=33
# UTM_BAND=V

# IP Addresses to connect to neptus
# The IP of the computer running neptus
NEPTUS_IP=192.168.2.69
# IP of SAM
SAM_IP=192.168.2.65
# Port for the imc-ros-bridge, usually doesnt change from 6002.
BRIDGE_PORT=6002


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
tmux send-keys "mon launch flexxros sam_controls.launch --name=$(tmux display-message -p 'p#I_#W')" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_drivers sam_core.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch dvl_ros_driver dvl.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:4
tmux send-keys "mon launch sam_dead_reckoning sam_gps_dr.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:5
tmux send-keys "mon launch sam_basic_controllers static_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:6
tmux send-keys "mon launch sam_basic_controllers dynamic_controllers.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:7
tmux send-keys "mon launch sam_dead_reckoning sam_gps_dummy.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:8
tmux send-keys "mon launch sam_mission mission.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND neptus_addr:=$NEPTUS_IP bridge_addr:=$SAM_IP bridge_port:=$BRIDGE_PORT --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

#tmux select-window -t $SESSION:9
#tmux send-keys "mon launch sam_communicator sam_communicator.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

#tmux select-window -t $SESSION:8
#tmux send-keys "roslaunch sam_drivers sam_monitor.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
