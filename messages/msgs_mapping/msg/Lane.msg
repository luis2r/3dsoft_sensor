#id and name
# for identification purpose
int32 id
std_msgs/String name

int32 road_id

#init
# (x,y,thetha,k,lambda,roff)
float64[] init

#final 
# (x,y,thetha,k,lambda,roff)
float64[] final

#sharp_length
#curvature derivate (sigma) and legth (s) 
geometry_msgs/Point[] sharp_length


#indexes
uint8 X=0        
uint8 Y=1        
uint8 THETHA=2   
uint8 KAPPA=3
uint8 LAMBDA=4
uint8 ROFF=5



