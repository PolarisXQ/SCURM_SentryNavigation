# cmd msg for controlling chassis
# constants for control type
uint8 VELOCITY = 1         # velocity control, (vx,vy,w) in chassis frame.
uint8 FOLLOW_GIMBAL = 2    # velocity control, follow gimbal's yaw stick automatically,
                           # (vx,vy) in gimbal's yaw frame.
uint8 SWING = 3            # velocity control, swing around gimbal's yaw automatically,
                           # (vx,vy) in gimbal's yaw frame.
uint8 SPIN = 4             # velocity control, spin at an angular velocity automatically,
                           # (vx,vy) in gimbal's yaw frame.

# control type
uint8 type 1

# control data
geometry_msgs/Twist twist