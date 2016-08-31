include <../settings.scad>;
use <plate.scad>;
use <countersink.scad>;
use <vertical.scad>;

motor_switch_offset_x = unit_thickness*4;
motor_switch_offset_y = (motor_plate_height-motor_switch_height_leads+2.5)/2;
motor_switch_wire_cutout = motor_switch_height_leads/3;
motor_switch_offset_y2 = (motor_plate_height-motor_switch_wire_cutout+2.5)/2;

module make_motor_switch_cutout(flip_y=0){

    u = unit_thickness;
    
    motor_switch_thickness = 6.5;
    
    body_z = flip_y ? -(motor_switch_thickness/2-tolerance*2) : (motor_switch_thickness/2-tolerance*2);

    // main switch body
    translate([motor_switch_offset_x,motor_switch_offset_y,body_z])
    cube([
        motor_switch_length+tolerance,//y
        motor_switch_height_leads+tolerance+2.5,//z
        motor_switch_thickness//x
    ], center=true);
    
    // hole for wires
    translate([motor_switch_offset_x,motor_switch_offset_y2,motor_switch_thickness/2])
    cube([motor_switch_length+tolerance, motor_switch_wire_cutout, motor_switch_thickness*2], center=true);
        
}

module make_motor_plate1(){
    u = unit_thickness;
    
    shd = motor_switch_hole_diameter;
    hspace = motor_switch_hole_spacing;

    union(){

        difference(){
            color("purple")
            cube([motor_plate_width-tolerance,motor_plate_height-tolerance,motor_plate_thickness], center=true);
            
            make_motor_switch_cutout();
            
            translate([-motor_offset_y,0,0])
            cylinder(d=motor_width+tolerance, h=100, center=true);
            
            make_motor_plate_middle(outline=1);
            
            make_motor_plate_join_holes(countersink=0);
        }
    
        //color("green"){
        
            translate([motor_switch_offset_x-hspace/2-shd/2,motor_switch_offset_y-motor_switch_height_leads/2+shd+motor_switch_hole_offset_z,0])
            rotate([0,0,0])
            cylinder(
                d=motor_switch_hole_diameter-tolerance,
                h=motor_plate_thickness,
                center=true);
                
            translate([motor_switch_offset_x+hspace/2+shd/2,motor_switch_offset_y-motor_switch_height_leads/2+shd+motor_switch_hole_offset_z,0])
            rotate([0,0,0])
            cylinder(
                d=motor_switch_hole_diameter-tolerance,
                h=motor_plate_thickness,
                center=true);
                
        //}
    
    }

}

module make_switch_holes(){
    u = unit_thickness;
    
    shd = motor_switch_hole_diameter;
    hspace = motor_switch_hole_spacing;
    tol = 0;//tolerance;
    offset_x = 0;//unit_thickness*3.5;
    offset_y = 0;
    offset_z = 0;//motor_switch_hole_offset_z;

    translate([offset_x-hspace/2-shd/2,offset_y-motor_switch_height_leads/2+shd+offset_z,0])
    rotate([0,0,0])
    cylinder(
        d=motor_switch_hole_diameter+tolerance,
        h=motor_plate_thickness*2,
        center=true);
        
    translate([offset_x+hspace/2+shd/2,offset_y-motor_switch_height_leads/2+shd+offset_z,0])
    rotate([0,0,0])
    cylinder(
        d=motor_switch_hole_diameter+tolerance,
        h=motor_plate_thickness*2,
        center=true);
        
}

module make_motor_plate2(){
    u = unit_thickness;
    
    shd = motor_switch_hole_diameter;
    hspace = motor_switch_hole_spacing;
    tol = 0;//tolerance;

    union(){
    
        difference(){
            color("purple")
            cube([motor_plate_width-tol,motor_plate_height-tol,motor_plate_thickness], center=true);
            
            translate([0,0,0])
            make_motor_switch_cutout(flip_y=1);
            
            translate([0,0,-u])//x,z,y
            make_motor_plate_middle(outline=1);
    
            make_motor_plate_join_holes(countersink=1);
        }
    
        //color("green"){
        
            translate([motor_switch_offset_x-hspace/2-shd/2,motor_switch_offset_y-motor_switch_height_leads/2+shd+motor_switch_hole_offset_z,0])
            rotate([0,0,0])
            cylinder(
                d=motor_switch_hole_diameter-tolerance,
                h=motor_plate_thickness,
                center=true);
                
            translate([motor_switch_offset_x+hspace/2+shd/2,motor_switch_offset_y-motor_switch_height_leads/2+shd+motor_switch_hole_offset_z,0])
            rotate([0,0,0])
            cylinder(
                d=motor_switch_hole_diameter-tolerance,
                h=motor_plate_thickness,
                center=true);
                
        //}
        
        //translate([+(horizontal_plate_width/2+unit_thickness/2), 0, bottom_clearance+vertical_plate_height/2-unit_thickness/2])
        //rotate([90,0,90])
        make_vertical_frame();
        
    }

}

module make_motor_plate_middle(outline=0){
    u = unit_thickness;
    
    // When we're getting the outline to cutout from other shapes, make sizes a little extra larger so the original shape has room to move.
    tol = outline ? tolerance: 0;
    
    offset_z = motor_plate_middle_thickness/2;
    angle = 5;
    d = motor_width + tolerance*4;
    
    outer_rim = 2.5;

    difference(){
        union(){
            hull(){
                
                // main motor mount
                translate([-motor_offset_y,0,offset_z])
                cylinder(d=d+tol, h=motor_plate_middle_thickness+tol, center=true);
                
                // spacer
                translate([wheel_axle_diameter*3,-(motor_width/2-wheel_axle_diameter*3/2),offset_z])
                cylinder(d=wheel_axle_diameter*2+tol, h=motor_plate_middle_thickness+tol, center=true);
                
                if(outline){
                    translate([-motor_offset_y,0,0])
                    rotate([0,0,-angle])
                    translate([+motor_offset_y,0,0])
                    translate([wheel_axle_diameter*3,-(motor_width/2-wheel_axle_diameter*3/2),offset_z])
                    cylinder(d=wheel_axle_diameter*2+tol, h=motor_plate_middle_thickness+tol, center=true);
                    
                    translate([-motor_offset_y,0,0])
                    rotate([0,0,-angle/2])
                    translate([+motor_offset_y,0,0])
                    translate([wheel_axle_diameter*3,-(motor_width/2-wheel_axle_diameter*3/2),offset_z])
                    cylinder(d=wheel_axle_diameter*2+tol, h=motor_plate_middle_thickness+tol, center=true);
                }
                
            }
        }
        
        // Switch cutout.
        if(outline==0){
            color("red"){
                translate([-2,1,0])
                scale([1.1,1,1])
                make_motor_switch_cutout();
            }
        }
        
        if(outline==0){
        
            // Motor mount hole.
            translate([-motor_offset_y,motor_mount_holes_dist/2,offset_z]){
                cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
            
                translate([0,0,5-u/2-1])
                make_countersink_motor();
            }
            
            // Motor mount hole.
            translate([-motor_offset_y,-motor_mount_holes_dist/2,offset_z]){
                cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
                
                translate([0,0,5-u/2-1])
                make_countersink_motor();
            }
            
            // motor axle cutout
            translate([-motor_offset_y,0,0])
            cylinder(d=motor_collar_width+tolerance, h=100, center=true);
            
            // wheel axle cutout
            translate([0,0,0])
            cylinder(d=motor_bearing_inner_diameter, h=100, center=true);
            
        }
    }
    
    if(outline){

        for(i=[-2:1:+1]){
            // wheel axle cutout
            translate([-motor_offset_y,0,0])
            rotate([0,0,angle*i])
            translate([+motor_offset_y,0,0])
            cylinder(d=motor_bearing_outer_diameter, h=100, center=true);
        }
        
        hull(){
        
            // wheel-facing cutout
            translate([-motor_offset_y,0,0])
            cylinder(d=d-outer_rim, h=100, center=false);
    
            // wheel axle cutout
            color("red"){
            for(i=[-5:1:+1]){
                translate([-motor_offset_y,0,0])
                rotate([0,0,angle*i])
                translate([+motor_offset_y,0,0])
                cylinder(d=motor_bearing_outer_diameter, h=100, center=false);
            }}
        }
    }
    
    
}

module make_motor_plate_join_holes(countersink=0){

    u = unit_thickness;
    rot_x = 0;
    rot_y = 0;
    rot_z = 0;
    cs_z = u*.4;
    
    color("red"){
    
        rotate([rot_x,rot_y,rot_z])
        translate([-motor_plate_width/2+u,motor_plate_height/2-u,0]){
            cylinder(d=screw_thread_diameter, h=100, center=true);
                
            if(countersink){
                translate([0,0,cs_z])
                make_countersink();
            }
        }
        
        rotate([rot_x,rot_y,rot_z])
        translate([motor_plate_width/2-u,-(motor_plate_height/2-u),0]){
            cylinder(d=screw_thread_diameter, h=100, center=true);
               
            if(countersink){
                translate([0,0,cs_z])
                make_countersink();
            }
        }
        
        rotate([rot_x,rot_y,rot_z])
        translate([-motor_plate_width/2+u,-motor_plate_height/2+u,0]){
            cylinder(d=screw_thread_diameter, h=100, center=true);
               
            if(countersink){
                translate([0,0,cs_z])
                make_countersink();
            }
        }
        
        rotate([rot_x,rot_y,rot_z])
        translate([u,motor_plate_height/2-u,0]){
            cylinder(d=screw_thread_diameter, h=100, center=true);
               
            if(countersink){
                translate([0,0,cs_z])
                make_countersink();
            }
        }
        
    }

}

module make_motor_unimount_1(){
    u = unit_thickness;
    tol = tolerance;
    
    main_h = motor_plate_height+u*0.5;
    main_offset_z = u*0.25;
    
    wheel_hub_offset = u*3;
    
    difference(){
        
        color("purple")
        union(){
            intersection(){
                //main body
                translate([0,-main_offset_z,0])
                cube([motor_plate_width, main_h, u], center=true);
                
                //circular right body curve to help rotation downward
                translate([-motor_plate_width/2-u/2,0,0])
                cylinder(d=motor_plate_width*2+u, h=u*2, center=true);
                
            }
        
            // upper refill
            translate([-u*1.0,u*3.5-u/2,0])
            cube([motor_plate_width-u*2., u, u], center=true);    
            
            // refill lower body curve
            translate([0,-(main_h)/4-main_offset_z,0])
            cube([motor_plate_width, (main_h)/2, u], center=true);
    
            // rotation pivot
            translate([-motor_plate_width/2-u/2,0,0]){
                difference(){
                    cylinder(d=screw_thread_diameter*4, h=u, center=true);
                    cylinder(d=screw_thread_diameter+tolerance, h=u*2, center=true);
                }
            }
            
            // tank wheel hub mount front
            hull(){
                translate([-(motor_plate_width/2+wheel_hub_offset/2),-main_h/2-wheel_hub_offset/2,0])
                rotate([0,0,0])
                cylinder(d=wheel_hub_offset, h=u, center=true);
                translate([-(motor_plate_width/2+wheel_hub_offset/2)+wheel_hub_offset,-main_h/2+wheel_hub_offset/2,0])
                rotate([0,0,0])
                cylinder(d=wheel_hub_offset, h=u, center=true);
            }
            
            // tank wheel hub mount front
            hull(){
                translate([(motor_plate_width/2+wheel_hub_offset/2),-main_h/2-wheel_hub_offset/2,0])
                rotate([0,0,0])
                cylinder(d=wheel_hub_offset, h=u, center=true);
                translate([(motor_plate_width/2+wheel_hub_offset/2)-wheel_hub_offset,-main_h/2+wheel_hub_offset/2,0])
                rotate([0,0,0])
                cylinder(d=wheel_hub_offset, h=u, center=true);
            }
            
        }
        
        // Upper hole cutouts.
        color("red"){
            for(i=[0:1:5]){
                hull(){
                    translate([-u*(i)*2+u*4,u*3.5,0])
                    cylinder(d=screw_head_diameter+tol, h=u*4, center=true);
                    translate([-u*(i)*2+u*4,u*3.5+u,0])
                    cylinder(d=screw_head_diameter+tol, h=u*4, center=true);
                }
            }
        }
        
        // Lower hole cutouts.
        color("red"){
            for(i=[0:1:4]){
                hull(){
                    translate([-u*(i)*2+u*4,-u*3.5,0])
                    cylinder(d=screw_head_diameter+tol, h=u*4, center=true);
                    translate([-u*(i)*2+u*4,-(u*3.5+u),0])
                    cylinder(d=screw_head_diameter+tol, h=u*4, center=true);
                }
            }
        }
        
        // Motor mount hole.
        translate([-motor_offset_y,motor_mount_holes_dist/2,offset_z]){
            cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
        
            translate([0,0,5-u/2-1])
            make_countersink_motor();
        }
        
        // Motor mount hole.
        translate([-motor_offset_y,-motor_mount_holes_dist/2,offset_z]){
            cylinder(d=motor_mount_holes_width, h=motor_plate_middle_thickness*2, center=true);
            
            translate([0,0,5-u/2-1])
            make_countersink_motor();
        }
        
        // motor axle cutout
        translate([-motor_offset_y,0,0])
        cylinder(d=motor_collar_width+tolerance, h=100, center=true);
        
        /*/ axle bearing cutout
        translate([0,0,motor_bearing_thickness])
        cylinder(d=motor_bearing_outer_diameter, h=motor_bearing_thickness*2, center=true);
        
        // wheel axle cutout
        translate([0,0,0])
        cylinder(d=motor_bearing_outer_diameter-2, h=motor_bearing_thickness*4, center=true);
        */
    
        translate([
            unit_thickness*3.5,//y
            unit_thickness,//z
            0
        ])
        make_switch_holes();
           
    }
    
}

module make_motor_unimount_restrainer(){
    u = unit_thickness;
    extra_t = 0.5;
    rotate([0,0,-90])
    translate([0,0,extra_t/2])
    scale([1,1,(extra_t+u)/u])
    difference(){
        union(){
        
            translate([-u,0,0])
            rotate([90,0,0])
            make_generic_beam(
                u=unit_thickness,
                holes=0,
                a=vertical_plate_height/u-4
            );
            
            translate([-u*3.5,-u*1+u*0.25,0])
            rotate([90,0,90])
            make_generic_beam(
                u=unit_thickness,
                holes=0,
                a=2.5
            );
            
            translate([-u/2,-u*.75,2.5+extra_t])
            cube([u*7,u*2.5,extra_t*2], center=true);
            
        }
        
        color("red"){
        
            translate([
                0,//z
                0,//y
                3//x
            ])
            make_countersink(l=u*3);
        /*    
            translate([
                u*2,//z
                0,//y
                3//x
            ])
            make_countersink(l=u*3);
          */  
            translate([
                -u*2,//z
                0,//y
                3//x
            ])
            make_countersink(l=u*3);
            
            translate([
                -u*3.5,//z
                -u*1.5,//y
                3//x
            ])
            make_countersink(l=u*3);
            
        }
    }
}

//make_motor_plate();
