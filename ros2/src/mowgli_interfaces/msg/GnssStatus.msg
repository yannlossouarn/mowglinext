uint8 FIX_TYPE_NO_FIX=0
uint8 FIX_TYPE_GPS_FIX=1
uint8 FIX_TYPE_RTK_FLOAT=2
uint8 FIX_TYPE_RTK_FIXED=3
uint8 FIX_TYPE_DEAD_RECKONING=4

uint32 CAP_POSITION_ACCURACY=1
uint32 CAP_HEADING=2
uint32 CAP_HEADING_ACCURACY=4
uint32 CAP_SATELLITES_USED=8
uint32 CAP_SATELLITES_VISIBLE=16
uint32 CAP_DIFFERENTIAL_STATUS=32
uint32 CAP_CORRECTION_AGE=64

std_msgs/Header header
uint8 fix_type
bool has_fix
bool differential_corrections
bool corrections_active
bool dead_reckoning
float32 quality_percent
uint32 capability_flags
float32 position_accuracy_m
float32 heading_deg
float32 heading_accuracy_deg
uint16 satellites_used
uint16 satellites_visible
float32 correction_age_s
