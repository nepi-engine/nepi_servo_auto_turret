## Pan and Tilt App status msg
## All data provided in the current reverse pan and tilt frames
## Reverse axis conversions are handled by the TMP IF class

#####################
# Pan Tilt Info
string[] available_pan_tilts
string selected_pan_tilt
bool pt_connected
string pt_connected_topic
nepi_interfaces/DevicePTXStatus pt_status_msg


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

bool pan_control_disabled
bool tilt_control_disabled

#################

bool scan_ready

bool scan_enabled
bool scan_pan_enabled
bool scan_tilt_enabled

bool is_scanning
bool pan_scanning
bool tilt_scanning


###################
string[] available_track_source_namespaces
string track_source_selected
bool track_source_connected
string track_image_topic
bool track_ready

float32 track_threshold
string[] track_best_filter_options
string track_best_filter

bool track_enabled
bool track_pan_enabled
bool track_tilt_enabled

bool is_tracking
bool pan_tracking
bool tilt_tracking

float32 track_pan_error
float32 track_tilt_error

###############
string[] available_stab_source_namespaces
string selected_stab_source
bool stab_source_connected

bool stab_ready

bool stab_enabled
bool stab_pan_enabled
bool stab_tilt_enabled

bool is_stabbing
bool pan_stabbing
bool tilt_stabbing


#********
# Auto Inputs
float32 auto_lat
float32 auto_long
bool auto_night_enabled

float32 auto_update_rate
float32 auto_move_ratio
string[] auto_control_names
float32[] auto_control_values

float32 auto_pan_min_deg
float32 auto_pan_max_deg
float32 auto_tilt_min_deg
float32 auto_tilt_max_deg


float32 roll_deg
float32 roll_dps

float32 pitch_deg
float32 pitch_dps

float32 yaw_deg
float32 yaw_dps

float32 heading_deg
float32 heading_dps


#********
# Auto outputs
string[] available_auto_processes
string selected_auto_process
bool auto_process_ready



float32 auto_pan_pos
float32 auto_tilt_pos
string auto_pos_display_title
float32 auto_pan_pos_display
float32 auto_tilt_pos_display
bool auto_pan_pos_disabled
bool auto_tilt_pos_disabled

float32 auto_pan_ratio_set
float32 auto_tilt_ratio_set
float32 auto_pan_ratio_display
float32 auto_tilt_ratio_display
bool auto_pan_ratio_disabled
bool auto_tilt_ratio_disabled

float32 auto_pan_speed_ratio_set
float32 auto_tilt_speed_ratio_set
bool auto_pan_speed_disabled
bool auto_tilt_speed_disabled

float32 auto_pan_deg
float32 auto_pan_adj
float32 auto_pan_goal
float32 auto_pan_dps
float32 auto_pan_pos_rate
float32 auto_pan_vel_rate

float32 auto_tilt_deg
float32 auto_tilt_adj
float32 auto_tilt_goal
float32 auto_tilt_dps
float32 auto_tilt_pos_rate
float32 auto_tilt_vel_rate
#********

#########################

bool click_pan_enabled
bool click_tilt_enabled

########################
# Selected Image Topics

string[] avail_image_topics
string[] selected_image_topics
string[] display_image_topics
string detect_source_topic
int32 single_image_index
# Number of Windows to Dipslay
int32 num_windows

bool full_screen_enabled



bool has_dual_mode
bool dual_mode_enabled
bool is_night
bool has_night_mode
bool night_mode_enabled
bool has_zoom_mode
bool zoom_mode_enabled
bool has_detect_mode
bool detect_mode_enabled
bool has_image_stab
bool image_stab_enabled

float32 stream_quality
float32 stream_rate
bool overlays_enabled


