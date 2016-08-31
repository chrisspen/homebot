include <settings.scad>;

module make_platform(){
    color("gray")
    translate([0,0,-5])
    //cube([max_footprint,max_footprint,10], center=true);
    cylinder(h=10, d=max_footprint, center=true, $fn=100);
}

make_platform();