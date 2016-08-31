include <../settings.scad>;
use <tray.scad>;
use <countersink.scad>;

module make_dc_converter_holes(height=20, d=3){
	offset_x = 53.5/2+3/2;//u*6.5;
	offset_y = 21/2;//u*1.5;
	translate([-offset_x,offset_y,0])
	cylinder(d=d, h=height, center=true);
	translate([-offset_x,-offset_y,0])
	cylinder(d=d, h=height, center=true);
	translate([offset_x,offset_y,0])
	cylinder(d=d, h=height, center=true);
	translate([offset_x,-offset_y,0])
	cylinder(d=d, h=height, center=true);
}


module make_dc_converter_tray(){
	d = screw_thread_diameter;
	height = 20;

	mount_x = u*5;
	mount_y = u*3.5;
	mount_z = 0.25;

	thickness = u*0.5;

	difference(){
		union(){
			cube([u*14, 40, thickness], center=true);
			//cube([80+u*2, u*4, thickness], center=true);
	
			translate([0,0,1.5])
			make_dc_converter_holes(d=5, height=5);
		}

        // mount holes for dc board
        make_dc_converter_holes(d=2);
        
        // mount holes of tray to frame
        for(i=[0:5]) for(j=[-1:2:1])
        translate([mount_x - 10*i,mount_y*j,0])
        color("green")
        make_countersink();
        
        // wire cutout
        color("red")
        for(j=[-1:2:1])
        translate([35*j,0,0])
        hull()
        for(i=[-1:2:1])
        translate([0,5*i,0])
        cylinder(d=10+2.5, h=height*2, center=true);
    
        // center bulk cutout
        color("red")
        hull()
        for(j=[-1:2:1])
        translate([15*j,0,0])
        for(i=[-1:2:1])
        translate([0,5*i,0])
        cylinder(d=10+2.5, h=height*2, center=true);
        
	}// end diff
    
}

make_dc_converter_tray();
