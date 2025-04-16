# cmd msg for controlling gimbal
# constants for control type
uint8 ABSOLUTE_ANGLE = 1    # position control, set position by absolute angle
uint8 RELATIVE_ANGLE = 2    # position control, set position by relative angle
uint8 VELOCITY = 3          # velocity control, set velocity

# control type
uint8 yaw_type 1
uint8 pitch_type 1

# control dada
Gimbal position
Gimbal velocity