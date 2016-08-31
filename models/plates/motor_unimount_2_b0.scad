include <../settings.scad>;
use <motor_unimount_2.scad>;
use <../spacers/spacers.scad>;
use <../nuts.scad>;
use <countersink.scad>;

module make_motor_unimount_2_b(){

    countersink_offset = u*0.25;

    difference(){
    
        make_motor_unimount_2(
            motor_holes=0,
            do_connector=1,
            do_countersink=1,
            do_grid_holes=1
            //,countersink_depth=-.75-u/2
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
        
        // powered gear tread cutout
        translate([-u*3, -u*1, -u*2]){
            difference(){
                color("blue")
                cylinder(d=u*7, h=u*3, center=true, $fn=100);
                translate([0,-u*7/2,0])
                cube([u*8,u*7,u*10], center=true);
            }
        }

    }//end diff
    
}

rotate([0,180,0])
make_motor_unimount_2_b();

/*
for(i=[-1:2:1])
translate([-u*6.5*i, -u*6, 3.5])
import("../printable/treads_flexiTreads_idler_20160330.stl");
*/