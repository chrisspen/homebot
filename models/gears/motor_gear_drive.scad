include <../settings.scad>;
//use <parametric_involute_gear_mcad_20150830.scad>;
use <parametric_involute_gear_v5.0.scad>;

module make_motor_gear_base(teeth=motor_gear_teeth){

    // Reduce this to reduce the size of the teeth.
    //circular_pitch = 360;
    circular_pitch = 180+1;
    height_b = 5;    
    twist = 200;
    for(i=[0:1:1])
    mirror([0,0,i])
    gear(
        number_of_teeth=teeth,
        circular_pitch=circular_pitch,
        gear_thickness = height_b/2,
        rim_thickness = height_b/2,
        hub_thickness = height_b/2,
        bore_diameter=0,
        twist=twist/teeth,
        circles=0
    );
    
}

module make_motor_gear_drive(teeth=motor_gear_teeth){
    notch_depth = 0.5;
    difference(){
        make_motor_gear_base(teeth=teeth);
        difference(){
            cylinder(d=track_driver_hole_radius*2, h=track_wheel_width*10, center=true);
            color("blue")
            translate([0,track_driver_hole_radius*3/2+track_driver_hole_radius-notch_depth,0])
            cube([track_driver_hole_radius*3,track_driver_hole_radius*3,track_wheel_width*20], center=true);
        }
    }
}

//make_motor_gear_base();

//make_motor_gear_drive();
make_motor_gear_drive(teeth=motor_gear_teeth);

if(0)
translate([0,0,-10])
color("green")
cylinder(d=30, h=5);
