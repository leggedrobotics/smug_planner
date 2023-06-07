# header.frame_id specifies the ToI's frame.
std_msgs/Header header
# The ToI's centroid in it's frame.
geometry_msgs/Point centroid
# A list of feasible contact points on the ToI, from which the Poses of Interest are calculated
geometry_msgs/Point[] contacts