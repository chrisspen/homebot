include <../settings.scad>;
use <horizontal.scad>;

module make_horizontal_frame_motor_top_slotted(){
	width = 80-u*2;
	height = 80+u*2;
	difference(){
		make_horizontal_frame(
            skip_i_a=1,
            skip_i_b=1,
            skip_i_c=1,
            skip_i_d=1
		);
        color("red")
        translate([0,height/2-u/2-tolerance/2,0])
        cube([width-u*2+tolerance, u*2, u*2], center=true);

	}
}

make_horizontal_frame_motor_top_slotted();
