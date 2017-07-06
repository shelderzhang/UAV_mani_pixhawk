uint8 arm_enable  # enable robotic arm  0 disenable, 1 enable  
int8 gripper_status # status of girpper 1 open, 0 closing, -1 closed 
float32 gripper_posi  
float32 x   # X Position in NED frame in meters  
float32 y   # Y Position in NED frame in meters  
float32 z  # Z Position in NED frame in meters (note, altitude is negative in NED)  
float32 roll   # roll setpoint in rad  
float32 pitch  # pitch setpoint in rad  
float32 yaw    # yaw setpoint in rad   
float32 vx  # X velocity in NED frame in meter / s  
float32 vy  # Y velocity in NED frame in meter / s  
float32 vz   # Z velocity in NED frame in meter / s  
float32 roll_rate   #  roll rate setpoint in rad/s  
float32 pitch_rate  #  pitch rate setpoint in rad/s  
float32 yaw_rate    #  yaw rate setpoint in rad/s 
