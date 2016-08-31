include <../settings.scad>;
use <plate.scad>;
use <motor.scad>;
use <countersink.scad>;

module make_motor_unimount_2(
    motor_holes=1,
    do_connector=0,
    do_countersink=1,
    do_grid_holes=0,
    countersink_depth=-u*.15,
    flip_cutout=0){
    
    u = unit_thickness;
    tol = tolerance;
    main_offset_z = u*1;

    through_depth = u*.25;
        
    countersink_d1 = do_connector ? (screw_thread_diameter+0.5) : 0;
    
    wheel_gap = track_wheel_gap;
    
    difference(){
        
        color("purple")
        union(){
        
            //main body
            translate([0,-main_offset_z,0])
            cube([motor_plate_width, motor_plate_height, u], center=true);
            
            // tank wheel hub mount front
            hull(){
                translate([
                    -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
                    -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
                    0
                ])
                cylinder(d=wheel_hub_offset, h=u, center=true);//lower
                translate([
                    -(motor_plate_width/2+wheel_hub_offset/2)+wheel_hub_offset,
                    -motor_plate_height/2+wheel_hub_offset/2,
                    0])
                cylinder(d=wheel_hub_offset, h=u, center=true);//upper
            }

            // tank wheel hub mount rear
            hull(){
                translate([
                    (motor_plate_width/2+wheel_hub_offset/2)+motor_arm_offset_y,
                    -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
                    0
                ])
                cylinder(d=wheel_hub_offset, h=u, center=true);//lower
                translate([(motor_plate_width/2+wheel_hub_offset/2)-wheel_hub_offset,-motor_plate_height/2+wheel_hub_offset/2,0])
                cylinder(d=wheel_hub_offset, h=u, center=true);//upper
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
        
        
        // tank wheel hub axle front
        translate([
            -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
            0
        ])
        cylinder(d=motor_unimount_idler_axle_d, h=100, center=true);
        
        // tank wheel hub axle rear
        translate([
            (motor_plate_width/2+wheel_hub_offset/2)+motor_arm_offset_y,
            -(motor_plate_height/2+wheel_hub_offset/2)-motor_arm_offset_z,
            0
        ])
        cylinder(d=motor_unimount_idler_axle_d, h=100, center=true);
        
        if(motor_holes){
            
            // Motor indent.
            translate([-motor_offset_y,motor_hole_height_offset,-u*1.25+1]){
                cylinder(d=motor_width+1, h=u*2, center=true);
            }
            
            // Motor mount hole.
            translate([-motor_offset_y,motor_mount_holes_dist/2+motor_hole_height_offset,offset_z]){
                cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
            
                translate([0,0,5-u/2-1.2])
                make_countersink_motor();
            }
            
            // Motor mount hole.
            translate([-motor_offset_y,-motor_mount_holes_dist/2+motor_hole_height_offset,offset_z]){
                cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
                
                translate([0,0,5-u/2-1.2])
                make_countersink_motor();
            }
            
            // motor axle cutout
            translate([-motor_offset_y,motor_hole_height_offset,0])
            cylinder(d=motor_collar_width+tolerance, h=100, center=true);
            
        }
        
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
        }
        
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

        // wheel buffer cutouts
        for(i=[-1:2:1])
        translate([-u*6.5*i, -u*6, ((flip_cutout==1)?3.5:-u*2.5-1)+u*1]){
            difference(){
                color("blue")cylinder(d=u*7.5, h=u*2+2+2, center=true);
                color("orange")cylinder(d=u*2-2, h=u*4, center=true);
            }
        }
        
    }//end diff
    
}

make_motor_unimount_2(
    do_connector=1,
    motor_holes=0,
    do_grid_holes=1
	//countersink_depth=-u*0.7
);

for(i=[-1:2:1])
translate([-u*6.5*i, -u*6, -u*2.5-1])
import("../printable/treads_flexiTreads_idler_20160330.stl");

color("blue")
translate([-u*3, -u*1, -u*2.75])
import("../printable/treads_flexiTreads_driver_20150801.stl");
