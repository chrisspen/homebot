use <lrf_calibration.scad>;

//rotate([0,0,90]){

for(i=[0:3])
translate([10*i,0,0])
make_lrf_centerpoint_nub();

for(i=[0:1])
translate([-10*i-10,0,0])
make_lrf_centerpoint_nub_top();
    
//}
