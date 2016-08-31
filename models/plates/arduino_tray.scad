include <../settings.scad>;
use <tray.scad>;
use <countersink.scad>;

module make_arduino_holes(height=20, d=3){
	translate([-8.875,33.45,0])
	cylinder(d=d, h=height, center=true);
	translate([+19.05,33.4,0])
	cylinder(d=d, h=height, center=true);
	translate([-24.15,-17.4,0])
	cylinder(d=d, h=height, center=true);
	translate([+24.15,-19,0])
	cylinder(d=d, h=height, center=true);
}

module make_arduino_mount_pegs(height=5, d=5){
	difference(){
		make_arduino_holes(height=5, d=5);
		make_arduino_holes(height=20, d=3);
	}
}


module make_plate_arduino_tray(){
    width = 70;
    depth = 95;
    tab_depth = 95-80;

	arduino_holes_y = 7;

    vent_width = 2;
    side_vents = 5;

    countersink_z = 1.5;

    battery_width = 44.5;

    difference(){
        union(){
            make_plate_tray(width=width, depth=depth, height=u*2, front=0);
        
            translate([-width/2,-depth/2+tab_depth+5,0])
            linear_extrude(height=u*2)
            polygon(points=[[0,0],[0,-20],[-u,-u*0.5],[-u,0]]);
        
            translate([width/2,-depth/2+tab_depth+5,0])
            mirror([1,0,0])
            linear_extrude(height=u*2)
            polygon(points=[[0,0],[0,-20],[-u,-u*0.5],[-u,0]]);

		    color("green")
			translate([0,arduino_holes_y,2.5])
			make_arduino_mount_pegs();

        }
    
        color("green")
        translate([-width/2-u/2,-depth/2+tab_depth-1,u])
        rotate([90,0,0])
        make_countersink();
    
        color("green")
        translate([width/2+u/2,-depth/2+tab_depth-1,u])
        rotate([90,0,0])
        make_countersink();

		color("blue")
    	translate([0,arduino_holes_y,0])
		make_arduino_holes();


		for(i=[0:4]){
			translate([0,-u*7+u*3*i-u/2,0])
			cylinder(d=u*2, h=u*4, center=true);
			if(i < 4){
				translate([-u*2.5,-u*7+u*3*i+u,0])
				cylinder(d=u*2, h=u*4, center=true);
				translate([u*2.5,-u*7+u*3*i+u,0])
				cylinder(d=u*2, h=u*4, center=true);
			}
		}

    }//end diff

}

/*
color("gray")
translate([0,7,5])
rotate([90,0,90])
scale([10,10,10])
import("../electronics/Arduino_UNO.stl");
*/

make_plate_arduino_tray();
