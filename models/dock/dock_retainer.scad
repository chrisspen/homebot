include <../settings.scad>;
use <dock.scad>;

difference(){
    make_dock_retainer();

    for(j=[-1:1:1])
    translate([j*19,0,0])
    rotate([0,0,-5*j])
    for(i=[0:2])
    translate([0,i*10-15,0])
    cylinder(d=7, h=20, center=true);
}
