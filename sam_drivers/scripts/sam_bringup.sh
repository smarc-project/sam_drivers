SESSION=sam_bringup

# Lidingo & Labbet
#UTM_ZONE=34
#UTM_BAND=V

# Kristineberg
UTM_ZONE=32
UTM_BAND=V

# Rest of Sweden
# UTM_ZONE=33
# UTM_BAND=V

# IP Addresses to connect to neptus
# The IP of the computer running neptus
NEPTUS_IP=192.168.2.222
# IP of SAM
SAM_IP=192.168.2.84
# Port for the imc-ros-bridge, usually doesnt change from 6002.
BRIDGE_PORT=6002

# For the camera, 0:down, 1:right, 2: left. 
# For now we only use one camera to avoid jetson  shutting down 
SENSOR_ID=0 
CAR_DEPTH=10

SSS_SAVE_PATH=/xavier_ssd/sidescan

# This is the workspace containing the ros packages that are needed

tmux -2 new-session -d -s $SESSION

tmux rename-window "roscore"
tmux new-window -t $SESSION:1 -n 'gui'
tmux new-window -t $SESSION:2 -n 'core'
tmux new-window -t $SESSION:3 -n 'dr'
tmux new-window -t $SESSION:4 -n 'static_ctrl'
tmux new-window -t $SESSION:5 -n 'dyn_ctrl'
#tmux new-window -t $SESSION:7 -n 'gps_dr'

tmux new-window -t $SESSION:6 -n 'actions'
#tmux new-window -t $SESSION:6 -n 'sam_monitor'
tmux new-window -t $SESSION:7 -n 'bt'
# tmux new-window -t $SESSION:8 -n 'camera'
tmux new-window -t $SESSION:8 -n 'rosbag'
tmux new-window -t $SESSION:9 -n 'payloads'

tmux select-window -t $SESSION:0
tmux send-keys "roscore" C-m

# start the gui and new_gui in one launch file 

tmux select-window -t $SESSION:1
# tmux send-keys "mon launch flexxros sam_controls.launch --name=$(tmux display-message -p 'p#I_#W')" C-m
tmux send-keys "roslaunch sam_drivers sam_gui.launch rosbridge_ip:=$SAM_IP namespace:=sam --wait" C-m

tmux select-window -t $SESSION:2
tmux send-keys "mon launch sam_drivers sam_core.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:3
tmux send-keys "mon launch sam_dead_reckoning sam_dr.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:4
tmux send-keys "mon launch sam_basic_controllers sam_control.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# tmux select-window -t $SESSION:5
# tmux send-keys "mon launch sam_action_servers sam_actions.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:5
tmux send-keys "mon launch bt_mission mission.launch utm_zone:=$UTM_ZONE utm_band:=$UTM_BAND neptus_addr:=$NEPTUS_IP bridge_addr:=$SAM_IP bridge_port:=$BRIDGE_PORT --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

#tmux send-keys "mon launch sam_camera_config enhance.launch sim:=false sensor_id:=$SENSOR_ID --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m
# tmux send-keys "mon launch sam_camera_config cameras.launch sim:=false --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m
#tmux send-keys "mon launch sam_camera_config record_multi_bash.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m
# tmux send-keys "mon launch sam_camera_config multi_nv_jpeg.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m
# tmux send-keys "mon launch sam_camera_config combined_camera.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:6
tmux send-keys "mon launch sam_drivers sam_rosbag.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

tmux select-window -t $SESSION:7
tmux send-keys "mon launch sam_drivers sam_payloads.launch sss_out_file:=$SSS_SAVE_PATH/ high_freq:=true range:=40 --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# hacky af, this sleep is.
# somehow the mon launch version doesnt work properly, so hack it is.
# sleep 5
# tmux select-window -t $SESSION:8
# tmux send-keys "roslaunch roswasm_webgui sam_webgui.launch rosbridge_ip:=$SAM_IP namespace:=sam" C-m
#tmux send-keys "mon launch roswasm_webgui sam_webgui.launch rosbridge_ip:=$SAM_IP namespace:=sam --name=$(tmux display-message -p 'p#I_#W')" C-m

#tmux select-window -t $SESSION:9
#tmux send-keys "mon launch sam_communicator sam_communicator.launch --name=$(tmux display-message -p 'p#I_#W') --no-start" C-m

# tmux send-keys "roslaunch sam_drivers sam_gui.launch rosbridge_ip:=$SAM_IP namespace:=sam" C-m

#tmux select-window -t $SESSION:8
#tmux send-keys "roslaunch sam_drivers sam_monitor.launch"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
