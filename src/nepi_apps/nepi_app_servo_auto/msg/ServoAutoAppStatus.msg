## Servo App status msg
## All data provided in the current reverse servo frames
## Reverse axis conversions are handled by the TMP IF class

#####################
# Servo Info
string[] available_servos
string selected_servo
bool sv_connected
string sv_connected_topic
nepi_interfaces/DeviceSVXStatus sv_status_msg


#unset values are -999
float32 servo_max_speed_dps
float32 servo_avg_move_delay

float32 speed_ratio
float32 pan_speed_ratio
float32 tilt_speed_ratio

float32 pan_deg
float32 tilt_deg

float32 pan_goal
float32 tilt_goal

float32 pan_deg_per_sec
float32 tilt_deg_per_sec



###################
# Auto Servo Info

bool pan_scanning
bool tilt_scanning

bool pan_tracking
bool tilt_tracking

bool pan_stabing
bool tilt_stabing

float32 pan_goto
float32 tilt_goto

bool pan_control_disabled
bool tilt_control_disabled

#################

bool scan_pan_enabled
bool scan_tilt_enabled

float32 scan_pan_speed_ratio
float32 scan_tilt_speed_ratio

float32 scan_pan_min_deg
float32 scan_pan_max_deg
float32 scan_tilt_min_deg
float32 scan_tilt_max_deg

float32 scan_pan_sec
float32 scan_tilt_sec

###############

###################

bool track_pan_enabled
bool track_tilt_enabled

float32 track_sv_min_speed_ratio
float32 track_sv_max_speed_ratio
float32 track_sv_max_speed_dps

float32 track_pan_min_deg
float32 track_pan_max_deg
float32 track_tilt_min_deg
float32 track_tilt_max_deg

bool track_source_connected

nepi_interfaces/TrackingStatus tracking_status

float32 track_goal_deg
float32 track_move_deg
float32 track_move_ratio
float32 track_reset_time_sec

float32 track_pan_error
float32 track_tilt_error


###############
string[] available_stab_source_namespaces
string selected_stab_source
bool stab_source_connected

string[] available_stab_processes
string selected_stab_process
bool stab_process_ready

bool stab_pan_ready
bool stab_tilt_ready

bool stab_pan_enabled
bool stab_tilt_enabled

float32 stab_pan_pos
float32 stab_tilt_pos

#********

float32 stab_update_rate
float32 stab_num_avg
float32 stab_move_ratio
float32 stab_reset_time_sec
string[] stab_control_names
float32[] stab_control_values

float32 stab_sv_min_speed_ratio
float32 stab_sv_max_speed_ratio
float32 stab_sv_max_speed_dps


float32 stab_roll_deg
float32 stab_roll_dps

float32 stab_pitch_deg
float32 stab_pitch_dps

float32 stab_heading_deg
float32 stab_heading_dps

float32 stab_pan_deg
float32 stab_pan_adj
float32 stab_pan_goal
float32 stab_pan_dps
float32 stab_pan_pos_rate
float32 stab_pan_vel_rate

float32 stab_tilt_deg
float32 stab_tilt_adj
float32 stab_tilt_goal
float32 stab_tilt_dps
float32 stab_tilt_pos_rate
float32 stab_tilt_vel_rate
#********

#########################

bool click_pan_enabled
bool click_tilt_enabled

########################
# Selected Image Topics
string[] image_topics
# Number of Windows to Dipslay
int32 num_windows
string[] image_priority_options


