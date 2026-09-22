# Periodic (5 Hz) state of the experiment runner.

string IDLE="IDLE"
string ARMED="ARMED"
string RUNNING="RUNNING"
string DONE="DONE"
string ABORTED="ABORTED"
string FAILED="FAILED"

std_msgs/Header header
string state
string run_id
string path             # straight / circle / square / s_curve
float64 t_run           # seconds since START
float64 t_total         # profile duration
int32 segment_index
string segment_label
float64 v_cmd           # m/s currently commanded
float64 w_cmd           # rad/s currently commanded
bool bag_active
bool preflight_ok
int32 repeat_index
int32 repeats
