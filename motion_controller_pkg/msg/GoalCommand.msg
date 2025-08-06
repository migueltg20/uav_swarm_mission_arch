# GoalCommand request

uint8 drone_id                              # from 0 to 3 (in my case, not necessarily)
geometry_msgs/PoseStamped  point            # goal
as2_msgs/TrajectorySetpoints trajectory     # a defined trajectory
bool circular                               # if the UAV have to make a circular trajectory around the goal
float32 radius                              # radius of the circular trajectory (in m)
as2_msgs/ControlMode control_mode           # control mode (look at as2_msgs/msg/ControlMode) (not functional with control_mode.yaw_mode=YAW_SPEED)