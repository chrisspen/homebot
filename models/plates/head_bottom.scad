include <../settings.scad>;
use <plate.scad>;
use <countersink.scad>;
use <cross_plate.scad>;
use <head_front.scad>;
use <../openscad-extra/fillet.scad>;

module make_head_top(){
    
	w = u*20;
	h = u*13;
	wo = 1 - (w % 2);
	ho = 1 - (h % 2);

	difference(){
	
	    make_cross_plate(
	        w=w,
	        h=h,
	        //hub_d=u*4,
	        t=u,
            cross_type=1
	    );
		
		// top/bottom face holes
			for(i=[0:2:w/u-1])for(j=[-1:2:1])
			translate([0, w/2-u/2-u*i-wo*u/2, (h/2-u/2)*j])
			rotate([0,90,0])
			color("red")
			cylinder(d=screw_thread_diameter, h=u*1.1, center=true);
		
		// top/bottom edge holes
			for(i=[0:2:w/u-3])for(j=[-1:2:1])
			translate([0, w/2-u/2-u*i-wo*u/2-u, (h/2-u/2)*j])
			rotate([0,0,0])
			color("red")
			cylinder(d=screw_thread_diameter, h=u*1.1, center=true);
	
		// front/back face holes
			for(i=[-1:2:1])for(j=[0:2:h/u-1])
			translate([0, (w/2-u/2)*i, h/2-u/2-u*j-ho*u/2+u])
			rotate([0,90,0])
			color("red")
			if(j!=0)
			cylinder(d=screw_thread_diameter, h=u*1.1, center=true);
	
		// front/back edge holes
			for(i=[-1:2:1])for(j=[0:2:h/u-1])
			translate([0, (w/2-u/2)*i, h/2-u/2-u*j-ho*u/2])
			rotate([90,90,0])
			color("red")
			//if(j!=0)
			cylinder(d=screw_thread_diameter, h=u*1.1, center=true);
        
        color("blue")
        translate([0,-45-2.5,0])
        cube([5+1,5+0.01,55], center=true);

	}
    /*
    for(i=[0:1]) for(j=[0:1])
    mirror([0,0,j])
    mirror([0,i,0])
    translate([0, -40, 22.5])
    rotate([0, 0, 90])
    make_2d_corner_fillet(width=10, height=10, depth=5);
    */
}

rotate([0, 90, 0])
make_head_top();
