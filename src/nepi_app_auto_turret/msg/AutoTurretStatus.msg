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
bool pan_control_manaul_enabled
bool tilt_control_manaul_enabled

bool pan_control_auto_enabled
bool tilt_control_auto_enabled

bool pan_control_disabled
bool tilt_control_disabled

float32 auto_pan_ratio
float32 auto_tilt_ratio

float32 auto_pan_deg
float32 auto_tilt_deg


##############
# Image Pub Info
string image_pub_topic

bool show_full_screen
bool show_targets_enabled
bool show_track_enabled
bool show_crosshair_enabled
float32[] crosshair_offset_degs
