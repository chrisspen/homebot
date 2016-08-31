include <../settings.scad>;
use <tray.scad>;
use <countersink.scad>;

module make_arduino_holes(height=20, d=2){
	translate([-8.875,33.45,0])
	cylinder(d=d, h=height, center=true);
	translate([+19.05,33.4,0])
	cylinder(d=d, h=height, center=true);
	translate([-24.15,-17.4,0])
	cylinder(d=d, h=height, center=true);
	translate([+24.15,-19,0])
	cylinder(d=d, h=height, center=true);
}

module make_arduino_mount_pegs(height=5, od=5, id=2){
	difference(){
		make_arduino_holes(height=height, d=od);
		make_arduino_holes(height=height*10, d=id);
	}
}


module make_plate_arduino_tray2(){
    width = 70;
    depth = 95;
    tab_depth = 95-80;

	arduino_holes_y = 7;

    vent_width = 2;
    side_vents = 5;

    countersink_z = 1.5;

    battery_width = 44.5;
    
    tray_depth_offset = -15;

    difference(){
        union(){
            translate([0, -tray_depth_offset/2, 0])
            make_plate_tray(width=width, depth=depth+tray_depth_offset, height=u*2, rail_width=1, front=0);
        
            translate([-width/2, -depth/2+tab_depth+5, 0])
            linear_extrude(height=u*2)
            polygon(points=[[0,0],[0,-u],[-u,-u],[-u,0]]);
        
            translate([width/2 + 0, -depth/2+tab_depth+5, 0])
            mirror([1,0,0])
            linear_extrude(height=u*2)
            polygon(points=[[0,0],[0,-u],[-u,-u],[-u,0]]);

		    color("green")
			translate([-5.5, 9, u*0.5/2+1.25])
			make_arduino_mount_pegs(height=3);

        }
        
		color("blue")
    	translate([-5.5, 9, 0])
		make_arduino_holes();
    
        // screw cutout
        color("green")
        for(i=[-1:2:1])
        translate([(-width/2-u/2)*i, -depth/2+tab_depth+1, u*1])
        rotate([90,0,0])
        make_countersink();
  
    }//end diff
    

}

/*
color("gray")
translate([-5.5, 9, 4])
rotate([90,0,90])
scale([10,10,10])
import("../electronics/Arduino_UNO.stl");
*/

make_plate_arduino_tray2();
