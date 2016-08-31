include <../settings.scad>;
use <plate.scad>;
use <../plates/countersink.scad>;
use <laser_test_mount.scad>;
use <../electronics/pi_noir_camera.scad>;

module make_head_face_sensor_tabs(offset=u){
    
    o1 = offset;
    o2 = offset + u*.5;
    
    difference(){
        union(){
            // screw tabs
            translate([0, o1+u/2, 0])
            cube([u*.5, u*1, u*2-tolerance/2], center=true);
            translate([0, -o1-u/2, u*2])
            cube([u*.5, u*1, u*2-tolerance/2], center=true);
            translate([0, -o1-u/2, -u*2])
            cube([u*.5, u*1, u*2-tolerance/2], center=true);
        }
    
        translate([u*0, o2, 0])
        rotate([0, 90, 0])
        make_countersink();
        
        translate([u*0, -o2, u*2])
        rotate([0, 90, 0])
        make_countersink();
        
        translate([u*0, -o2, -u*2])
        rotate([0, 90, 0])
        make_countersink();
    }
}

module make_head_face_laser_mount(){
    translate([u*2,0,0]){
        
        rotate([180,0,0])
        translate([u*.5,0,-u*2])
        rotate([90,0,90])
        make_laser_test_mount(show_parts=0, show_camera_holes=0);
    
        translate([u*.75,0,0])
        make_head_face_sensor_tabs(offset=u*2.5);
        
    }
}

module make_led_mount(){
    
    difference(){
        union(){
            // main body
            cube([u*.5, u*2, u*7], center=true);
            
            make_head_face_sensor_tabs();
        }
        
        // led cutouts, 5mm holes
        translate([0,0,0])
        rotate([0,90,0])
        cylinder(d=5, h=u*2, center=true);
        translate([0,0,u*2])
        rotate([0,90,0])
        cylinder(d=5, h=u*2, center=true);
        translate([0,0,-u*2])
        rotate([0,90,0])
        cylinder(d=5, h=u*2, center=true);
    }
    
}

module make_head_face_sensor_mount(){

    height = u*9;
    depth = u*2;
    rim_depth = u*.5;
    
    difference(){
    
    union(){
        color("red"){
            for(i=[-1:2:1]){
                translate([depth/2 - u/2, (u*5/2 + u/2)*i, 0])
                cube([depth, u*1, height], center=true);
                
                translate([depth/2 - u/2, (u*11/2 + u/2)*i, 0])
                cube([depth, u*1, height], center=true);
            }
        }
        
        difference(){
            translate([rim_depth/2-u*1/2,0,0])
            cube([rim_depth, u*13, height], center=true);
            
            translate([0,0,0])
            cube([u*2, u*13-u*2, height-u*1], center=true);
        }
    }
    
    for(i=[0:4]){for(j=[-1:2:1]){
        translate([((i==0 || i==4)?u*1.3:u*2), u*6*j, u*4-u*2*i])
        rotate([0,90,0])
        make_countersink(inner=u*3, outer=u*3);
        
        translate([u*2, u*3*j, u*4-u*2*i])
        rotate([0,90,0])
        make_countersink(inner=u*3, outer=u*3);
    }}
    
    }//end diff
    
}


import("../printable/head_front_20150930.stl");

translate([u,0,0])
make_head_face_sensor_mount();

//color("green")
/*
translate([u*2,0,0])
rotate([180,0,0])
translate([u*.5,0,-u*2])
rotate([90,0,90])
make_laser_test_mount(show_parts=0);*/
make_head_face_laser_mount();

color("blue")
translate([u*2.75, u*4.5, 0])
make_led_mount();
