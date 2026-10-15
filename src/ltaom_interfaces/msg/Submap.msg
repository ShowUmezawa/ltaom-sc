# Atomic submap identity + payload. IDs are explicit, never header.seq.
uint32 schema
string session_id
uint64 submap_id
uint64 first_scan_id
uint64 last_scan_id
builtin_interfaces/Time association_time
geometry_msgs/Pose anchor_pose
sensor_msgs/PointCloud2 cloud
string semantic_sha256
