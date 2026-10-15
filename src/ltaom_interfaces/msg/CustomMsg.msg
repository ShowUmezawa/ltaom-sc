# Livox CustomMsg fields; used only to compile FAST-LIO's unused Livox path.
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points

