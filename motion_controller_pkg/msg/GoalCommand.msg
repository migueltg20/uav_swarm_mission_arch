# GoalCommand request

uint8 drone_id                              # from 0 to 3 (in my case, not necessarily)
float32 takeoff                             # takeoff altitude (in m)
bool land                                   # land command
geometry_msgs/PoseStamped  point            # goal
as2_msgs/TrajectorySetpoints trajectory     # a defined trajectory
bool circular                               # if the UAV have to make a circular trajectory around the goal
float32 radius                              # radius of the circular trajectory (in m)
bool cancel                                 # 1 to cancel the previous goal request (if it hasn't finished yet)