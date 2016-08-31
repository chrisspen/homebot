include <../settings.scad>;

use <../openscad-extra/wedge.scad>;

use <plate.scad>;
use <../plates/countersink.scad>;
use <../plates/neck.scad>;
use <../plates/head.scad>;
use <../plates/head_bottom.scad>;
use <../plates/head_top.scad>;
use <../plates/head_front.scad>;

use <../electronics/screen.scad>;
use <../electronics/camera.scad>;
use <../electronics/slipring.scad>;
use <../electronics/motor.scad>;
use <../electronics/BMS706MG_Servo.scad>;

use <../bearings/slew_bearing.scad>;

use <../gears/pan_gears.scad>;

module make_neck_struct_bottom_holes(cs=0){
    depth = 1;
    outer_depth = u;
    d2 = screw_head_diameter + tolerance;
    
    color("blue"){
/*
        rotate([90,0,0]){
            translate([-u*2,0,0]){
                cylinder(d=screw_thread_diameter, h=10, center=true);
                translate([0,0,u*.8])make_countersink(outer=outer_depth, inner=depth);
            }
            translate([u*2,0,0]){
                cylinder(d=screw_thread_diameter, h=10, center=true);
                translate([0,0,u*.8])make_countersink(outer=outer_depth, inner=depth);
            }
        }
*/

        rotate([90,0,0])
        translate([u*3,u*1,0]){
            cylinder(d=screw_thread_diameter, h=10, center=true);
            translate([0,0,u*.8])make_countersink(outer=outer_depth, inner=depth, d2=d2);
        }

        rotate([90,0,0])
        translate([u*3,u*3,0]){
            cylinder(d=screw_thread_diameter, h=10, center=true);
            translate([0,0,u*.8])make_countersink(outer=outer_depth, inner=depth, d2=d2);
        }

        rotate([90,0,0])
        translate([-u*3,u*1,0]){
            cylinder(d=screw_thread_diameter, h=10, center=true);
            translate([0,0,u*.8])make_countersink(outer=outer_depth, inner=depth, d2=d2);
        }

        rotate([90,0,0])
        translate([-u*3,u*3,0]){
            cylinder(d=screw_thread_diameter, h=10, center=true);
            translate([0,0,u*.8])make_countersink(outer=outer_depth, inner=depth, d2=d2);
        }
        
    }
}

module make_neck_strut_side_holes(height=0, do_top=1, do_side=1){

    height = height ? height : neck_strut_height;

    // broad face holes
    for(i=[0:neck_strut_width/u-1]){
        for(j=[0:height/u-1]){
            if((i % 2 && j % 2) || (!(i % 2) && !(j % 2))){
                translate([i*u - neck_strut_width/2 + u/2, j*u - height/2 + u/2, 0])
                cylinder(d=screw_thread_diameter, h=10, center=true);
            }
        }
    }

    // top/bottom holes
    if(do_top){
    for(i=[0:neck_strut_width/u-1]){
        for(j=[0:1]){
            if(!((i % 2 && j % 2) || (!(i % 2) && !(j % 2)))){
                translate([i*u - neck_strut_width/2 + u/2, height*j - height/2, 0])
                rotate([90, 0, 0])
                cylinder(d=screw_thread_diameter, h=10, center=true);
            }
        }
    }
    }

    // left/right holes
    if(do_side){
        for(i=[0:2:height/u-1]){
            for(j=[0:1]){
                //if(!((i % 2 && j % 2) || (!(i % 2) && !(j % 2)))){
                    translate([neck_strut_width*j - neck_strut_width/2, i*u - height/2 + u/2 + u, 0])
                    rotate([0, 90, 0])
                    cylinder(d=screw_thread_diameter, h=10, center=true);
                //}
            }
        }
    }

}

module make_neck_strut(){
    height_extra = 15;
    difference(){
        union(){
            difference(){
                
                union(){
                    translate([0,-height_extra/2,0])
                    cube([neck_strut_width, neck_strut_height+height_extra, u], center=true);    
                }
                
                color("blue"){                            
                    // bottom wiring hole
                    translate([0, -neck_strut_height/2 + u/2 -height_extra, 0])
                    cube([3*u,u*3,u*2], center=true);
                }
                
            }//end diff

            // side supports
            translate([neck_strut_width/2 - u/2, -neck_strut_height/2 + u*1 -height_extra, u*2])
            cube([u, u*2, 3*u], center=true);
            translate([-(neck_strut_width/2 - u/2), -neck_strut_height/2 + u*1 -height_extra, u*2])
            cube([u, u*2, 3*u], center=true);
            
        
        }

        // side support curve fillet
        color("blue")
        translate([0, -neck_strut_height/2 + u*3 - u*1 -height_extra, u*3.5])
        rotate([0,90,0])
        scale([1,2/3/2,1])
        cylinder(d=6*u, h=u*8, center=true);

        // bottom screw mount holes
        translate([0, -neck_strut_height/2 -height_extra, 0])
        make_neck_struct_bottom_holes();
    
    }// end diff
}

module make_neck_strut_ring_holes(){

    y_offset2 = u*22 - neck_strut_height;

        translate([0, y_offset2*0.5+u*.5, 0]){
            color("red"){
                for(i=[-1:2:1]){
                    translate([u*4*i,u*10,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*5*i,u*9,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*6*i,u*8,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*7*i,u*7,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*8*i,u*6,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*9*i,u*5,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*9*i,u*3,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*9*i,u*1,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*9*i,-u*1,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*8*i,-u*2,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*7*i,-u*3,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*6*i,-u*4,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*5*i,-u*5,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                    translate([u*4*i,-u*6,0])
                    cylinder(d=screw_thread_diameter, h=20, center=true);
                }
            }
        }
}

module make_angle_notches(){

    big_nick = 0.5;
    small_nick = 0.5;

        color("yellow")
        for(i=[-1:2:1]){
            for(theta=[-45:5:45]){
                rotate([0, 0, theta])
                //translate([i*(u*10 + big_nick + ((theta % 10)?small_nick:0)), 0, 0])
                translate([i*(u*10 + small_nick), 0, 0])
                rotate([0,0,45])
                cube([2,2,u*2], center=true);
            }
        }
}

module make_neck_strut_servo(holes=1){

    // sizing for the length of holes
    height_extra = 20;
    height = u*23 + height_extra;
    y_offset = height - neck_strut_height;

    y_offset_center = 22.5;

    difference(){
        union(){
            make_neck_strut();
        
            difference(){
                color("Chocolate")
                translate([0,20+2.5,0])
                rotate([0,0,0])
                cylinder(
                    d=u*20,
                    h=u,
                    center=true, $fn=100);
            
                // center cutout
                color("lightblue")
                translate([0,20+2.5,0])
                rotate([180,0,0])
                wedge(h=u*2, r=u*8, a=270);
            }
        }

        if(holes){
            
            translate([0, y_offset*0.5 - height_extra, 0]){
                color("blue")
                make_neck_strut_side_holes(height=height, do_top=0);
            }
            
            make_neck_strut_ring_holes();
    
            /*
            translate([0, y_offset_center, 0])
            make_angle_notches();
            */
        }
    }//end diff

}

module make_neck_strut_open(holes=1){

    // sizing for the length of holes
//    y_offset = height - neck_strut_height;
    y_offset = u*(4*5)+u*3;

    y_offset_center = 22.5;

    difference(){
        union(){
            difference(){
                union(){
                    make_neck_strut();
        
                    color("Chocolate")
                    translate([0,y_offset_center,0])
                    rotate([0,0,0])
                    cylinder(
                        d=u*20,
                        h=u,
                        center=true, $fn=50);
                }

                // center cutout
                color("lightblue")
                translate([0,y_offset_center,0])
                rotate([180,0,0])
                wedge(h=u*2, r=u*8, a=270);

                // top expansion holes
                color("blue")
                translate([0,-u*2*7+5,0])
                rotate([0,0,180])
                make_neck_strut_side_holes();

            }// end diff
            
            // center hub
            translate([0,y_offset_center,0])
            cylinder(d=u*7, h=u, center=true);
        }

        if(holes){

            // axle hole
            translate([0,y_offset_center,0])
            cylinder(d=screw_thread_diameter+tolerance, h=u, center=true);
            
            // bottom expansion holes
            color("blue")
            translate([0, y_offset*0.5, 0])
            make_neck_strut_side_holes(height=u*5, do_top=0, do_side=0);
    
            make_neck_strut_ring_holes();
    
            // measurement notches
            translate([0, y_offset_center, 0])
            make_angle_notches();
            
        }
        
        // pivot hole
        color("red")
        translate([0,y_offset_center,-u*.3])
        rotate([0,180,0])
        cylinder(d=u*5, h=u*3, center=true);
    
        // centering notch
        translate([0, y_offset_center + u*2.45, 0])
        rotate([0,0,45])
        cube([2+0.5,2+0.5,20], center=true);
        
        // wire strain gauge mount hole
        color("blue"){
    
            translate([0,5+2.5,0])
            rotate([0,90,0])
            cylinder(d=2.5, h=15, center=true);
            
            for(i=[-1:2:1])
            translate([7.5*i,5+2.5,0])
            scale([2,1,1])
            cylinder(d=2.5, h=50, center=true);
        }

    }// end diff
    
}

module make_neck_strut_servo_mount(n=2){
    mount_offset_x = -u*1;

    translate([mount_offset_x,5,u*5.6])
    cube([u,7.5,u*1.25], center=true);
    translate([mount_offset_x,-5,u*5.6])
    cube([u,7.5,u*1.25], center=true);
    translate([mount_offset_x+u*.25,0,u*5.6])
    cube([u*.5,7.5*2,u*1.25], center=true);

    if(n==2){
        translate([mount_offset_x,5,u*15.5])
        cube([u,7.5,u*1.25], center=true);
        translate([mount_offset_x,-5,u*15.5])
        cube([u,7.5,u*1.25], center=true);
        translate([mount_offset_x+u*.25,0,u*15.5])
        cube([u*.5,7.5*2,u*1.25], center=true);
    }
}

module make_neck_strut_servo2(show_servo=0, show_mounts=0, show_holes=1){

    //mount_offset_x = -u*1;
    servo_offset_x = 0.76;
            
    if(show_servo){
        translate([0, 0, 65]){
           
            // head vertical rotation axis
            color("green")
            translate([0,0,0])
            rotate([0,90,0])
            cylinder(d=2, h=200, center=true);
                        
        }
    }

    difference(){

        union(){
            //color("brown")
            translate([0,0,42.5]){
                
                // neck right
                //translate([-neck_strut_spacing,0,0])
                rotate([90,0,90])
                //make_neck_strut();
                make_neck_strut_servo(holes=show_holes);
                //import("../printable/plates_plate_neck_servo_mock_20150903.stl");

            }

            // fill in unnecessary holes
            translate([0,0,u*12])
            cube([u, u*7, u*14.5], center=true);

            // servo mounts
            if(show_mounts)
            make_neck_strut_servo_mount();

        }

        // vertical rotator servo
        translate([-20+2.75-servo_offset_x,0,65])
        color("orange")
        rotate([0,-90,180])
        scale([1.05,1.05,1])
        make_BMS706MG_servo(horn=1, d_tolerance=tolerance*2);

        // vertical rotator servo holes
        translate([-10,0,65-(43/2-9.3)])
        color("orange")
        rotate([0,-90,180])
        make_BMS706MG_servo_holes(d=screw_thread_diameter);

        // cutout to allow servo horn to pass
        translate([-20+2.75,0,65])
        color("blue")
        rotate([0,-90,180])
        translate([0,0,30-7.75])
        rotate([180,0,0])
        make_BMS706MG_servo_horn_round(bb=1, d_extra=35);

        
    }// end diff

    if(show_servo){
        // vertical rotator servo
        translate([-20+2.75-servo_offset_x,0,65])
        color("orange")
        rotate([0,-90,180])
        make_BMS706MG_servo(horn=1);
    }

}

//make_neck_strut();

//rotate([0,-90,0]) make_neck_strut_servo2(show_servo=0, show_mounts=0, show_holes=1);

//make_neck_struct_bottom_holes();

//make_neck_strut_servo();

//make_neck_strut_open(holes=1);

make_neck_strut_servo(holes=1);
