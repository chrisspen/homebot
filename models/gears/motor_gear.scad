include <../settings.scad>;
use <parametric_involute_gear_v5.0.scad>;

module make_motor_gear(p){
    gear(number_of_teeth=17,
        circular_pitch=p,
        circles=8);
}

module make_motor_gears(){
	p = 185;
	rotate([180,0,0])
	translate([motor_offset_y/2,0,0]){
	
	    translate([-motor_offset_y,0,0])
	    make_motor_gear(p);
	    
	    //make_motor_gear(p);
	}
}

make_motor_gears();
