include <../settings.scad>;
use <tray.scad>;
use <countersink.scad>;
use <../electronics/battery.scad>;

module make_plate_battery_tray(){
	width = 70;
	depth = 95;
	tab_depth = 95-80;

	vent_width = 2;
	side_vents = 5;

	countersink_z = 1.5;

	battery_width = 44.5;

	difference(){
		union(){
		    make_plate_tray(width=width, depth=depth, height=u*2);
		
			translate([-width/2,-depth/2+tab_depth+5,0])
			linear_extrude(height=u*2)
			polygon(points=[[0,0],[0,-20],[-u,-u*0.5],[-u,0]]);
		
			translate([width/2,-depth/2+tab_depth+5,0])
			mirror([1,0,0])
			linear_extrude(height=u*2)
			polygon(points=[[0,0],[0,-20],[-u,-u*0.5],[-u,0]]);
	
			color("blue")
			translate([-0.5 - battery_width/2,0,u])
			cube([1, depth-u*6, u*2], center=true);

			color("blue")
			translate([-(-0.5 - battery_width/2),0,u])
			cube([1, depth-u*6, u*2], center=true);

			// join pillars
			color("green"){
				translate([-width/2+u/2,-depth/2+u*3.5,u])
				cube([u,u,u*2], center=true);
				translate([-(-width/2+u/2),-depth/2+u*3.5,u])
				cube([u,u,u*2], center=true);
				translate([-width/2+u/2,-(-depth/2+u*3.5),u])
				cube([u,u,u*2], center=true);
				translate([-(-width/2+u/2),-(-depth/2+u*3.5),u])
				cube([u,u,u*2], center=true);
			}
		}
	
        // frame mount holes
		color("green")
		translate([-width/2-u/2,-depth/2+tab_depth-1,u])
		rotate([90,0,0])
		make_countersink(outer=u*4);
		color("green")
		translate([width/2+u/2,-depth/2+tab_depth-1,u])
		rotate([90,0,0])
		make_countersink(outer=u*4);
	
		translate([width/2 - 8.5, -depth/2, u*2])
		rotate([90,0,0])
		cylinder(d=12, h=u*2, center=true);
	
		mirror([1,0,0])
		translate([width/2 - 8.5, -depth/2, u*2])
		rotate([90,0,0])
		cylinder(d=12, h=u*2, center=true);

		// join holes
		translate([-width/2+u/2,-depth/2+u*3.5,countersink_z])
		rotate([180,0,0])
		make_countersink();
		translate([-(-width/2+u/2),-depth/2+u*3.5,countersink_z])
		rotate([180,0,0])
		make_countersink();
		translate([-width/2+u/2,-(-depth/2+u*3.5),countersink_z])
		rotate([180,0,0])
		make_countersink();
		translate([-(-width/2+u/2),-(-depth/2+u*3.5),countersink_z])
		rotate([180,0,0])
		make_countersink();

		translate([0,0,u*1.75])
		cube([width*.75, u*4.5, u*2], center=true);

		for(i=[0:side_vents]){
			translate([0,-side_vents*vent_width+i*vent_width*2,u*1.75])
			cube([width, vent_width, u*2], center=true);
		}

		for(i=[0:5]){
			translate([0,-u*7+u*3*i-u/2,0])
			cylinder(d=u*2, h=u*4, center=true);
			if(i!=5){
				translate([-u*2.5,-u*7+u*3*i+u,0])
				cylinder(d=u*2, h=u*4, center=true);
				translate([u*2.5,-u*7+u*3*i+u,0])
				cylinder(d=u*2, h=u*4, center=true);
			}
		}

	color("orange")
	translate([0,0,9.5])
	rotate([0,0,90])
	make_battery_dc12300();

	}//end diff

}

make_plate_battery_tray();
