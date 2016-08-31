include <../settings.scad>;
use <motor_unimount_2.scad>;
use <../spacers/spacers.scad>;
use <../nuts.scad>;

module make_motor_unimount_2_a(){

    difference(){
        make_motor_unimount_2(do_countersink=0, flip_cutout=1);
        
/*
        translate([
            -motor_plate_width/2+wheel_hub_offset/2,
            -u/2-motor_plate_height/2-wheel_hub_offset/2,
            nylon_spacer_length/2+u/2-motor_unimount_spacer_depth
        ])
        make_motor_nylon_spacer();
        
        translate([
            -(-motor_plate_width/2+wheel_hub_offset/2),
            -u/2-motor_plate_height/2-wheel_hub_offset/2,
            nylon_spacer_length/2+u/2-motor_unimount_spacer_depth
        ])
        make_motor_nylon_spacer();
  */  
        translate([
            -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
            -u/2
        ])
        make_nut_2_5(d=0);
        
        translate([
            (motor_plate_width/2+wheel_hub_offset/2)+motor_arm_offset_y,
            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
            -u/2
        ])
        make_nut_2_5(d=0);
        
    }//end diff
}

//intersection(){
rotate([0,180,0])
make_motor_unimount_2_a();

//translate([-u*6.5,-u*6,u*2])cube([u*2,u*2,u*4],center=true);
//}