# High power circuit health-related values
# EscState ... { voltage: float64, water_leaked: bool }

float64 voltage      # [V]
float64 current      # [A]
float64 temperature  # [degC]
bool    water_leaked
EscState esc_lf_state
EscState esc_lb_state
EscState esc_rb_state
EscState esc_rf_state
