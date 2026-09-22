# Discrete event in an experiment run. Published latched (transient_local) by the
# runner and recorded in the bag and in data/runs/<run_id>/events.csv.

string PREFLIGHT_OK="PREFLIGHT_OK"
string ARMED="ARMED"
string BAG_STARTED="BAG_STARTED"
string START="START"
string SEGMENT="SEGMENT"
string END="END"
string ABORT="ABORT"
string FAIL="FAIL"
string MANUAL_MOVE="MANUAL_MOVE"
string ZERO="ZERO"
string NOTE="NOTE"
string BAG_STOPPED="BAG_STOPPED"

std_msgs/Header header
string run_id
string event            # one of the constants above
string detail           # segment label, abort reason, note text, ...
int32 repeat_index      # 0-based; -1 when not applicable
int32 segment_index     # 0-based; -1 when not applicable
float64 t_run           # seconds since START (negative before START)
