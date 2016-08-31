include <../settings.scad>;
use <motor_unimount_2.scad>;
use <../spacers/spacers.scad>;
use <../nuts.scad>;
use <countersink.scad>;

module make_motor_unimount_2_b(countersink_type=1){

    countersink_offset = u*0.25;

	//punchout_offset = 0.025;//too high
	//punchout_offset = 0.075;//too high
	punchout_offset = 0.125;
	//punchout_offset = 0.15;//too low

    difference(){
    
        make_motor_unimount_2(
            motor_holes=0,
            do_connector=1,
            do_countersink=1,
            do_grid_holes=1,
            countersink_depth=-.75-u/2
        );
        /*
        translate([
            -motor_plate_width/2+wheel_hub_offset/2,
            -u/2-motor_plate_height/2-wheel_hub_offset/2,
            -(nylon_spacer_length/2+u/2)+motor_unimount_spacer_depth
        ])
        make_motor_nylon_spacer();
        
        translate([
            -(-motor_plate_width/2+wheel_hub_offset/2),
            -u/2-motor_plate_height/2-wheel_hub_offset/2,
            -(nylon_spacer_length/2+u/2)+motor_unimount_spacer_depth
        ])
        make_motor_nylon_spacer();
        */

		if(countersink_type == 1){

        	translate([
	            -motor_plate_width/2+wheel_hub_offset/2,
	            -u/2-motor_plate_height/2-wheel_hub_offset/2,
	            countersink_offset
	        ])
	        make_countersink();
	    
	        translate([
	            -(-motor_plate_width/2+wheel_hub_offset/2),
	            -u/2-motor_plate_height/2-wheel_hub_offset/2,
	            countersink_offset
	        ])
	        make_countersink();

		}

		if(countersink_type == 2){

            translate([
	            -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
	            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
				track_nut_height*2 - 1 -track_nut_height
	        ])
	        make_nut_2_5(d=0, h=track_nut_height*2);
	        
	        translate([
	            (motor_plate_width/2+wheel_hub_offset/2)+motor_arm_offset_y,
	            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
				track_nut_height*2 - 1 -track_nut_height
	        ]){
	        make_nut_2_5(d=0, h=track_nut_height*2);
			}

		}


    }//end diff
    
	// add punchout to hex countersinks?
	if(countersink_type == 2){

		color("green")
		translate([
            -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
				-u/2-track_punchout_thickness + (u-track_nut_height) + punchout_offset
		])
		cube([u*2,u*2,track_punchout_thickness], center=true);

		color("green")
		translate([
            (motor_plate_width/2+wheel_hub_offset/2)+motor_arm_offset_y,
            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
				-u/2-track_punchout_thickness + (u-track_nut_height) + punchout_offset
		])
		cube([u*2,u*2,track_punchout_thickness], center=true);

	}
}

rotate([0,180,0])
make_motor_unimount_2_b(countersink_type=2);
