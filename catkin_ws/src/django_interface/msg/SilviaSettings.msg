std_msgs/Header header
float64 temperature_setpoint
float64 heater_kp
float64 heater_kd
float64 heater_ki

float64 mass_setpoint

uint16 t_update
uint16 t_sample

uint16 t_clean_on
uint16 t_clean_off
uint16 n_clean_cycles

float64[] profile_pressure_setpoints
float64[] profile_time_setpoints
float64 pump_kp
float64 pump_kd
float64 pump_ki