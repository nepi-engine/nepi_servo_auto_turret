# This message details the AutoTurret Status Msg

string node_name
string namespace

string[] data_products
string save_data_topic

float32 max_process_rate_hz
float32 max_image_pub_rate_hz


###############################
bool auto_select_enabled

string selected_pantilt_topic
bool pantilt_connected
nepi_interfaces/DevicePTXStatus pantilt_status_msg


string selected_image_topic
bool image_connected
nepi_interfaces/ImageStatus image_status_msg


string selected_targets_topic
bool targets_connected
nepi_interfaces/TargetsStatus targets_status_msg

string selected_navpose_topic
bool navpose_connected
nepi_interfaces/NavPoseStatus navpose_status_msg

###############################
# The auto process is the supervisor: it owns the pan/tilt control loop that
# the scan, track and stabilize modes select between. Nothing auto-drives an
# axis unless auto is enabled.

bool auto_ready
bool auto_enabled
string auto_process_namespace

bool scanning_ready
bool scanning_enabled
string scan_process_namespace

bool tracking_ready
bool tracking_enabled
string track_process_namespace

bool stabilize_ready
bool stabilize_enabled
string stab_process_namespace


#############################
# Pan Tilt Axis Info

nepi_app_auto_turret/AutoTurretAxis auto_pan_axis
nepi_app_auto_turret/AutoTurretAxis auto_tilt_axis

##############
# Image Pub Info
string image_pub_topic

bool show_full_screen
bool show_targets_enabled
bool show_track_enabled
bool show_goal_enabled
bool image_stab_enabled

