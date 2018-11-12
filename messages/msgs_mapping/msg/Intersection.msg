
#id and name
# for identification purpose
int32 id
std_msgs/String name

#coordinate of the center of the node
geometry_msgs/Point center  

#size of internal and external radius in case of roundabouts 
# it must be in m (S.I)
float32 external_radius 
float32 internal_radius

# type of the node
uint8 ROUNDABOUT=0
uint8 CROSSING=1

int32 type

# intersections in
int32[] roads_in

# intersections out
int32[] roads_out




