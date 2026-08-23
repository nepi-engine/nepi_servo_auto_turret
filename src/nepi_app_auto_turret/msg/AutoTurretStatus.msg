# This message details the AutoTurret Status Msg

nepi_interfaces/ProcessStatus process_status

nepi_interfaces/DevicePTXStatus pt_status_msg

string[] avaiable_image_topics
string selected_image_topic

string[] avaiable_detector_topics
string selected_detector_topic

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
bool show_track_enabled = msg.show_track_enabled
bool show_crosshair_enabled = msg.show_crosshair_enabled
float32[] crosshair_offset_degs
