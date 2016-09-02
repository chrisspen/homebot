include <../settings.scad>;

use <../openscad-extra/wedge.scad>;
use <flexiTreads_idler.scad>;
use <flexiTreads_driver.scad>;

module mock_tread(){
    
    wheel_radius = track_wheel_radius;
    thickness = 2.5;
    tread_length = 65;
    depth = 10;
    
    // right
    r1_angle = 150;
    r1_length = 70;
    
    // left
    r2_angle = 125;
    r2_length = 30;
    
    // top
    r3_angle = 106;
    r3_length = 60;
    
    /*
    // bottom
    translate([wheel_radius-thickness/2, -50/2, 0])
    cube([thickness, tread_length, 20], center=true);
    
    // right
    rotate([0, 0, 150])
    translate([wheel_radius-thickness/2 + 2, 20, 0])
    cube([thickness, tread_length, 20], center=true);
    */
    
    // right
    translate([0, 10, 0])
    rotate([0, 0, -15]){
        
        rotate([0, 0, (180-r1_angle)/2])
        translate([wheel_radius-thickness/2, -r1_length/2, 0])
        cube([thickness, r1_length, depth], center=true);
        
        color("blue")
        difference(){
            wedge(h=depth , r=wheel_radius, a=r1_angle);
            translate([0,-0.01,0]) wedge(h=depth+1, r=wheel_radius-thickness, a=r1_angle);
        }
    }
    
    // left
    translate([0, -60, 0])
    rotate([0, 0, -150]){
        
        rotate([0, 0, (180-r2_angle)/2])
        translate([wheel_radius-thickness/2, -r2_length/2, 0])
        cube([thickness, r2_length, depth], center=true);
        
        color("blue")
        difference(){
            wedge(h=depth , r=wheel_radius, a=r2_angle);
            translate([0,-0.01,0]) wedge(h=depth+1, r=wheel_radius-thickness, a=r2_angle);
        }
    }
    
    // top
    translate([-26, -43, 0])
    rotate([0, 0, -243]){
        
        rotate([0, 0, (180-r3_angle)/2])
        translate([wheel_radius-thickness/2, -r3_length/2, 0])
        cube([thickness, r3_length, depth], center=true);
        
        color("blue")
        difference(){
            wedge(h=depth , r=wheel_radius, a=r3_angle);
            translate([0,-0.01,0]) wedge(h=depth+1, r=wheel_radius-thickness, a=r3_angle);
        }
    }
    
}

translate([-track_wheel_radius, 50/2, 50+1])
color("black")
mock_tread();

if(0)
rotate([0,-90,0]){

    echo("wheel(lr).y:", -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y);
    echo("wheel(lr).z:", -wheel_hub_offset/2+bottom_clearance);

    // left 1
        translate([
            horizontal_plate_width/2+u*2 + track_wheel_width_buffer,
            -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
            -wheel_hub_offset/2+bottom_clearance
        ])
    rotate([0,90,0])
    make_track_idler();
    // left 2
        translate([
            -(horizontal_plate_width/2+u*2 + track_wheel_width_buffer +u*2),
            -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
            -wheel_hub_offset/2+bottom_clearance
        ])
    rotate([0,90,0])
    make_track_idler();

    // right 1
        translate([
        horizontal_plate_width/2+u*2 + track_wheel_width_buffer,
            +(motor_plate_width/2+wheel_hub_offset/2+motor_arm_offset_y),
            -wheel_hub_offset/2+bottom_clearance
        ])
    rotate([0,90,0])
    make_track_idler();
    // right 2
        translate([
        -(horizontal_plate_width/2+u*2 + track_wheel_width_buffer +u*2),
            +(motor_plate_width/2+wheel_hub_offset/2+motor_arm_offset_y),
            -wheel_hub_offset/2+bottom_clearance
        ])
    rotate([0,90,0])
    make_track_idler();

    echo("wheel(c).y:", -motor_offset_y);
    echo("wheel(c).z:", bottom_clearance + motor_plate_height/2 + motor_hole_height_offset/2);

    // center 1
        translate([
            horizontal_plate_width/2+u*2 + track_wheel_width_buffer,
            -motor_offset_y,
            bottom_clearance + motor_plate_height/2 + motor_hole_height_offset/2
        ])
    rotate([0,90,0])
    make_track_driver();
    // center 2
        translate([
            -(horizontal_plate_width/2+u*2 + track_wheel_width_buffer +u*2),
            motor_offset_y,
            bottom_clearance + motor_plate_height/2 + motor_hole_height_offset/2
        ])
    rotate([0,90,0])
    make_track_driver();
}
