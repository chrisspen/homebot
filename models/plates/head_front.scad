include <../settings.scad>;
use <plate.scad>;
use <../plates/countersink.scad>;
use <../plates/cross_plate.scad>;

module make_head_front(){
    
	w = u*13;
	h = u*13;

	difference(){
	    make_cross_plate(
	        w=w,
	        h=h,
	        //hub_d=u*4,
	        t=u
	    );
	
		for(i=[0:2:w/u-1])for(j=[0:2:h/u-1])
		translate([0, w/2-u/2-u*i, h/2-u/2-u*j])
		rotate([0,90,0])
		color("red")
		if(i==0 || j==0 || i==w/u-1 || j==h/u-1)
		cylinder(d=screw_thread_diameter, h=u*4, center=true);
	
		for(theta=[0:1])
	    for(i=[0:2:w/u-1])for(j=[0:2:h/u-2])
		rotate([90*theta,0,0])
	    translate([0, w/2-u/2-u*i, j*u-h/2+u*1.5])
	    rotate([90,0,0])
	    color("red")
	    if(i==0 || i==w/u-1)
	    cylinder(d=screw_thread_diameter, h=u*1.1, center=true);

        // bottom cutout for SD card
        color("blue")
        translate([0,0,-30-1/2+0.05])
        cube([5+1,55,5+1], center=true);
	    
	}
}

rotate([0,90,0])
make_head_front();
