include <../settings.scad>;
use <dock.scad>;

difference(){
    translate([0,2.5,-75])
    rotate([90,0,0])
    make_dock_mast_head();

    for(i=[-1:2:1])
    for(j=[-1:2:1])
    translate([15*i,15*j+16.5,0])
    cylinder(d=1, h=10, center=true);
}
