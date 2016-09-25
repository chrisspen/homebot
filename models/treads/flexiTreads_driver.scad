include <../settings.scad>;
use <flexiTreads.scad>;
use <../nuts.scad>;

module make_track_driver(set_screw=1){

	notch_depth = 0.5;

	difference(){
        difference(){
			rotate([0,0,90])
		    if(track_wheel_show_fancy){		
                wheel(track_wheel_radius, track_wheel_width, track_wheel_offset, false);
		    }else{
		        cylinder(d=track_wheel_radius*2, h=track_wheel_width, center=false);
		    }

			difference(){
	            cylinder(d=track_driver_hole_radius*2, h=track_wheel_width*10, center=true);
				color("blue")
				translate([0,track_driver_hole_radius*3/2+track_driver_hole_radius-notch_depth,0])
				cube([track_driver_hole_radius*3,track_driver_hole_radius*3,track_wheel_width*20], center=true);
			}

        }

        // set screw cutout
        if(set_screw)
        translate([0,track_driver_hole_radius+2.5,track_wheel_width/2]){
            rotate([90,0,0]){
                color("red")
                hull(){
                    make_nut_2(w=6, d=0);
                    translate([0,10,0])
                    make_nut_2(w=6, d=0);
                }
            }
            color("green")
            translate([0,track_wheel_radius/2-4,0])
            rotate([90,0,0])
            cylinder(d=track_driver_set_screw_shaft_d, h=track_wheel_radius, center=true);
            
            color("blue")
            translate([0,track_wheel_radius/2-1,0])
            rotate([90,0,0])
            cylinder(d=track_driver_set_screw_head_d, h=track_wheel_radius/2, center=true);
        }

	}// end diff


}

make_track_driver(set_screw=0);
