#Forces reading.

Header header           # timestamp is the time the depth was measured
                         # frame_id is the location of the depth reading

float64 surge     # Measurement of the Surge Force in N
float64 sway     # Measurement of the Sway Force in N
float64 heave     # Measurement of the Heave Force in N
float64 yaw     # Measurement of the Yaw Force in Nm

float64 surge_ref     # Measurement of the Surge Ref Force in N
float64 sway_ref     # Measurement of the Sway Ref Force in N
float64 heave_ref     # Measurement of the Heave Ref Force in N
float64 yaw_ref     # Measurement of the Yaw Ref Force in Nm


float64 variance        # 0 is interpreted as variance unknown