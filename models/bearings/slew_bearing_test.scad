include <../settings.scad>;
use <slew_bearing.scad>;

//import("../printable/bearings_slew_bearing_outer_20150926.stl");

intersection(){
	color("red")
	translate([0,0,0])
	import("../printable/bearings_slew_bearing_inner_top_20150927.stl");
	
	cylinder(d=head_slew_bearing_inner_diameter-tolerance, h=10, center=true, $fn=100);
}