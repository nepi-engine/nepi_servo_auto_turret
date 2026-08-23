## Auto Turret App status msg

#####################
# Pan Tilt Info
string[] available_pan_tilts
string selected_pan_tilt
bool pt_connected
string pt_connected_topic
nepi_interfaces/DevicePTXStatus pt_status_msg


#unset values are -999
float32 pan_tilt_max_speed_dps


float32 speed_ratio
float32 pan_speed_ratio
float32 tilt_speed_ratio

float32 pan_goto
float32 tilt_goto

float32 pan_deg
float32 tilt_deg

float32 pan_goal
float32 tilt_goal

float32 pan_deg_per_sec
float32 tilt_deg_per_sec

bool pan_manual_disabled
bool tilt_manual_disabled

#################

bool scan_ready
bool scan_enabled
bool is_scanning

###################
string[] available_track_source_namespaces
string track_source_selected
bool track_source_connected
string track_image_topic
bool track_ready
bool track_enabled
bool is_tracking

float32 track_threshold
string[] track_best_filter_options
string track_best_filter

float32 track_pan_error
float32 track_tilt_error

###############
string[] available_stab_source_namespaces
string selected_stab_source
bool stab_source_connected
bool stab_ready
bool stab_enabled
bool is_stabbing

float32 stab_pan_error
float32 stab_tilt_error

#########################

bool click_pan_enabled
bool click_tilt_enabled

########################
# Selected Image Topics

string[] avail_image_topic
string[] selected_image_topic
string[] display_image_topic

bool full_screen_enabled

bool has_detect_mode
bool detect_mode_enabled

bool has_image_stab
bool image_stab_enabled

float32 stream_quality
float32 stream_rate



