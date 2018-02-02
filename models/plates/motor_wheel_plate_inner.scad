
//module make_motor_wheel_plate_inner(){
//    cube([80, 5, 40], center=true);
//}

include <../settings.scad>;
use <plate.scad>;
use <motor.scad>;
use <../openscad-extra/src/countersink.scad>;
use <countersink.scad>;
use <motor_wheel_axle.scad>;

module make_motor_wheel_plate_inner(
    motor_holes=1,
    do_connector=0,
    do_countersink=1,
    do_grid_holes=0,
    countersink_depth=-u*.15,
    flip_cutout=0,
    motor_offset_x=-4
    ){
    
    u = unit_thickness;
    tol = tolerance;
    main_offset_z = u*1;

    through_depth = u*.25;
        
    countersink_d1 = do_connector ? (screw_thread_diameter+0.5) : 0;
    
    wheel_gap = track_wheel_gap;
    
    motor_hole_offset_z = 1.1;

    difference(){
        translate([0,0,5])
        rotate([90,0,0])
        difference(){
            
            union(){
            
                //main body
                color("purple")
                translate([0,-main_offset_z,0])
                cube([motor_plate_width, motor_plate_height, u], center=true);
                
                //bottom curve to support wheel axle
                translate([0,-20,0])
                //rotate([90,0,0])
                cylinder(d=25, h=5, center=true, $fn=100);
                
                rotate([-90,0,0])translate([0,0,-5]){
                    // side panels
                    for(i=[-1:2:1])
                    translate([(80/2-1/2)*i,-12/2-5/2,0])
                    cube([1, 12, 40], center=true);
                    
                    // top panel
                    translate([0,-12/2-5/2,40/2-5/2])
                    cube([80, 12, 5], center=true);
                }
                
                if(do_connector){
                
                    // top right
                    color("orange")
                    translate([(u*(8.5-1))/2, motor_plate_height/2-main_offset_z/2-u, -wheel_gap/2-u/2])
                    cube([u*8.5,u,wheel_gap], center=true);
                        
                    // top left
                    color("orange")
                    //translate([-u*6.75, motor_plate_height/2-main_offset_z/2-u,  -wheel_gap/2-u/2])
                    translate([(-u*(8.5-1))/2, motor_plate_height/2-main_offset_z/2-u, -wheel_gap/2-u/2])
                    cube([u*8.5,u,wheel_gap], center=true);
                    
                    // top left side
                    color("orange")
                    translate([-u*7.5, motor_plate_height/2-main_offset_z/2-u*1.75,  -wheel_gap/2-u/2])
                    cube([u,u*2.5,wheel_gap], center=true);
                    
                    // top right side
                    color("orange")
                    translate([u*7.5, motor_plate_height/2-main_offset_z/2-u*2.75,  -wheel_gap/2-u/2])
                    cube([u,u*4.5,wheel_gap], center=true);
                    
                    // bottom
                    color("orange")
                    translate([(u*(0))/2, -motor_plate_height/2-main_offset_z/2, -wheel_gap/2-u/2])
                    cube([u*5,u,wheel_gap], center=true);
                
                }
                

            }// end union
            
            translate([motor_offset_x,0,0])
            if(motor_holes){
                
                // Motor indent.
                translate([-motor_offset_y,motor_hole_height_offset,-u*1.25+1]){
                    cylinder(d=motor_width+1, h=u*2, center=true);
                }
                
                // Motor mount hole.
                translate([-motor_offset_y,motor_mount_holes_dist/2+motor_hole_height_offset,motor_hole_offset_z]){
                    cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
                
                    translate([0,0,5-u/2-1.2])
                    make_countersink_motor();
                }
                
                // Motor mount hole.
                translate([-motor_offset_y,-motor_mount_holes_dist/2+motor_hole_height_offset,motor_hole_offset_z]){
                    cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
                    
                    translate([0,0,5-u/2-1.2])
                    make_countersink_motor();
                }
                
                // motor axle cutout
                translate([-motor_offset_y,motor_hole_height_offset,0])
                cylinder(d=motor_collar_width+tolerance, h=100, center=true);
                
            }
            
            
            /*
            //top row
            for(i=[0:1:6]){
                translate([
                    motor_plate_width/2-u*2-i*u*2,
                    motor_plate_height/2-main_offset_z-u/2,
                    0
                ]){
                    cylinder(d=screw_thread_diameter, h=u*4, center=true);
                    
                    if(do_countersink)
                    color("red")translate([0,0,2+countersink_depth+(i==1||i==5 ? -through_depth : 0)])make_countersink(d1=countersink_d1, d2=screw_head_diameter+(i==1||i==5 ? .5 : 0));
                }
            }
            // bottom row
            for(i=[0:1:2]){
                translate([
                    motor_plate_width/2-u*2-i*u*2-u*4,
                    -(motor_plate_height/2-u/2)-main_offset_z,
                    0
                ]){
                    cylinder(d=screw_thread_diameter, h=u*4, center=true);
                    
                    if(do_countersink)
                    color("red")translate([0,0,2+countersink_depth+(i==1 ? -through_depth : 0)]){
                        make_countersink(d1=countersink_d1, d2=screw_head_diameter+(i==1 ? .5 : 0));
                    }
                }
            }
            // right
            for(i=[0:1:1]){
                translate([
                    motor_plate_width/2-u/2,
                    motor_plate_height/2-u*2-i*u*2-main_offset_z,
                    0
                ]){
                    cylinder(d=screw_thread_diameter, h=u*4, center=true);
                    
                    if(do_countersink)
                    color("red")translate([0,0,2+countersink_depth])make_countersink(d1=countersink_d1);
                }
            }
            // left
            for(i=[0:1:0]){
                translate([
                    -(motor_plate_width/2-u/2),
                    motor_plate_height/2-u*2-i*u*2-main_offset_z,
                    0
                ]){
                    cylinder(d=screw_thread_diameter, h=u*4, center=true);
                    
                    if(do_countersink)
                    color("red")translate([0,0,2+countersink_depth])make_countersink(d1=countersink_d1);
                }
            }*/
            
            if(do_grid_holes){
                for(ii=[0:1:6]){
                    for(jj=[0:1:2]){

                        translate([
                            -(motor_plate_width/2-u*2*ii-u*2),//x
                            motor_plate_height/2-u*2*jj-u*2-main_offset_z,//z
                            0
                        ]){
                            cylinder(d=screw_thread_diameter, h=u*4, center=true);
                            
                            //if(do_countersink)
                            //color("red")translate([0,0,2])make_countersink();
                        }

                    }
                }
            }
            
        }//end diff

        // Axle cutout
        //translate([0,-(12+5)/2,-15])rotate([90,0,0])cylinder(d=8+0.5, h=12+5, center=true, $fn=100);
        translate([0,-(12+5)/2,-15])rotate([0,0,0])make_motor_wheel_axle_cutout();
        
        // chassis attachment screw holes top
        for(i=[-1:2:1])
        color("red")
        translate([(80/2-10)*i,-14,20-2.5])
        rotate([90,0,0])
        make_countersink(d1=2.5+0.5, d2=5, inner=20);
    
        // chassis attachment screw holes bottom
        for(i=[-1:2:1])
        color("red")
        translate([(80/2-10)*i,-2,-(20-2.5)])
        rotate([90,0,0])
        make_countersink(d1=2.5+0.5, d2=5, inner=20);
        
        // outer plate attachment screw holes
        for(i=[-1:2:1])
        color("red")
        translate([(80/2-10-10)*i,-20,20-2.5])
        rotate([90,0,0])
        make_countersink(d1=2.5, d2=5, inner=50);
        
        // Axle set screw hole
        color("red")translate([0,2.5,-15])rotate([-90,0,0])make_countersink(d1=2.5, d2=5, inner=20);
        
    }//end diff

}

rotate([0,0,90])
rotate([-90,0,0])
make_motor_wheel_plate_inner();
