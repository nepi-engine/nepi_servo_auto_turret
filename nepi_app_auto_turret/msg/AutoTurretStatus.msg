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
nepi_interfaces/TargetingStatus targets_status_msg

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


bool pan_control_manaul_enabled
bool tilt_control_manaul_enabled

bool pan_control_auto_enabled
bool tilt_control_auto_enabled

bool pan_control_disabled
bool tilt_control_disabled


#############################
float32 auto_pan_goal_ratio
float32 auto_tilt_goal_ratio

float32 auto_pan_goal_deg
float32 auto_tilt_goal_deg

float32 auto_pan_error_deg
float32 auto_tilt_error_deg

##############
# Image Pub Info
string image_pub_topic

bool show_full_screen
bool show_targets_enabled
bool show_track_enabled
bool show_goal_enabled

