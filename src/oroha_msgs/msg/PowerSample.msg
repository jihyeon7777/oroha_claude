# One measurement window from the Pico firmware ("D," line, 50 Hz / 20 ms) plus
# the host-side conversion that was applied to it. Raw ADC values are kept so the
# conversion can be redone later with a different calibration.

std_msgs/Header header                 # stamp = host receive time (ROS clock); frame_id = "oroha_power"
builtin_interfaces/Time device_stamp   # firmware t_us mapped onto the ROS clock (min-filter offset)
uint32 seq                             # firmware sample counter (restarts on 'S' / reboot)
int64  t_us                            # firmware monotonic microseconds as sent (can go backwards/negative after a long idle: firmware ticks_diff wrap)
uint16 n_rounds                        # ADC rounds averaged in this window

# raw 12-bit ADC: window mean / min / max. int16 on purpose: the firmware can
# emit -1 / out-of-range sentinels for a window that collected no samples.
float32 gp26_mean                      # bus-voltage divider, GP26 / ADC0
int16   gp26_min
int16   gp26_max
float32 gp27_mean                      # current sensor #2 = slave id 2 = LEFT wheels, GP27 / ADC1
int16   gp27_min
int16   gp27_max
float32 gp28_mean                      # current sensor #1 = slave id 1 = RIGHT wheels, GP28 / ADC2
int16   gp28_min
int16   gp28_max
uint8   flags                          # b0-2 low rail (V/GP27/GP28), b3-5 high rail, b6 zero_valid, b7 overrun
bool    zero_valid
bool    overrun

# converted values (host side; see calib_id)
float32 v_bus                          # V, motor bus voltage
float32 i_left                         # A, LEFT MD400 DC-input branch; discharge positive
float32 i_right                        # A, RIGHT MD400 DC-input branch; discharge positive
float32 p_left                         # W = v_bus * i_left
float32 p_right                        # W = v_bus * i_right
float32 p_total                        # W = p_left + p_right

# conversion inputs actually used for this sample
float64 zero_gp27                      # raw at 0 A applied to GP27
float64 zero_gp28                      # raw at 0 A applied to GP28
float64 rail_corr                      # firmware rail back-estimate ratio (1.0 if unavailable)
float64 sync_offset_s                  # ROS time - t_us (s) at the min-filter estimate
float32 sync_residual_ms               # receive time - (t_us + offset) for this sample
string  calib_id                       # e.g. "sensing-20260828"
string  fw_version                     # e.g. "oroha-bench-1.1" (from "#CFG fw=")
