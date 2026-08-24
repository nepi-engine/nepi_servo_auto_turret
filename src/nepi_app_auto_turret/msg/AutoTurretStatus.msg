# This message details the AutoTurret Status Msg

nepi_interfaces/ProcessStatus process_status

nepi_interfaces/DevicePTXStatus pt_status_msg

# Source selection lives on the connect interfaces, not here. Each of
# <app>/ptx_connect, <app>/idx_connect and <app>/targets_connect publishes its
# own available list, selection and connection state, and the RUI renders those
# rows from Nepi_IF_ConnectPTX / Nepi_IF_ConnectIDX / Nepi_IF_ConnectTargets.
#
# selected_image_topic stays because it is a parent-to-child contract, not a RUI
# field: auto_turret_app_img_pub_node.py reads it to pick the image it overlays.
string selected_image_topic

bool has_navpose
string navpose_topic
bool navpose_topic_connected

bool scanning_ready
bool scanning_enabled

bool tracking_ready
bool tracking_enabled

bool stabilize_ready
bool stabilize_enabled

bool pan_control_disabled
bool tilt_control_disabled



#unset values are -999
float32 pan_tilt_max_speed_dps
float32 pan_tilt_avg_move_delay

float32 speed_ratio
float32 pan_speed_ratio
float32 tilt_speed_ratio

float32 pan_deg
float32 tilt_deg

float32 pan_goal
float32 tilt_goal

float32 pan_deg_per_sec
float32 tilt_deg_per_sec

float32 pan_goto
float32 tilt_goto



##############
# Image Pub Info
string image_pub_topic

bool show_full_screen
bool show_targets_enabled
bool show_track_enabled
bool show_crosshair_enabled
float32[] crosshair_offset_degs
