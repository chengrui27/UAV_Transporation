std_msgs/Header header

# Map parameters
float64[3] map_origin
float64 resolution
int32[3] map_voxel_num

# Local bound (valid data range)
int32[3] local_bound_min
int32[3] local_bound_max

# Shared memory name
string shm_name
