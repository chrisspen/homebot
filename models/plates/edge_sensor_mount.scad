include <../settings.scad>;
use <../electronics/pololu_ir_sensor.scad>;
use <../electronics/switch_d2f_01l83.scad>;
use <countersink.scad>;

module make_edge_sensor_mount_bb(width, height, thickness=0){
    intersection(){
        color("orange")
        cylinder(d=150-thickness*2, h=height, center=true, $fn=100);
        
        color("blue")
        translate([0, u*12, 0])
        //cube([width-thickness*2, u*8-thickness*2, height*2], center=true);
        cube([width, u*8, height*2], center=true);
    }
}

module make_edge_sensor_mount(show_sensors=1, show_holes=1){

    height = u*4;
    thickness = 2;
    //width = u*16;//just inner box
    width = u*18;//inner box and first unit of motor mount
    //width = u*26;//entire front
    sensor_angle = 30;
    sensor_dist = u*13;
    mount_offset_z = height/2-u/2;//-thickness/2;
    
    spring_d = 3.75;
    
    middle_sensor_offset1 = 0;
    middle_sensor_offset2 = -1;

    difference(){
        intersection(){
            union(){
                difference(){
    
                    union(){
                        
                    
                        // sensor mounting bulkhead
                        intersection(){
                            translate([0,0,mount_offset_z])
                            //make_edge_sensor_mount_bb(width=width, height=height+thickness);
                            make_edge_sensor_mount_bb(width=width-1*2, height=height, thickness=0);
    
                            for(i=[-1:1:1]){
                                rotate([0, 0, sensor_angle*i])
                                translate([0, sensor_dist, u*2])
                                translate([(i==0)?middle_sensor_offset1:0, 0, 0])
                                //rotate([0, 90+45, 90])
                                rotate([-90-45, 0, 0])
                                rotate([0,0,(i==1)?180:0]){
                                    // sensor reads "on" as long as there's something within 5cm of sensor
                                    //make_pololu_GP2Y0D805Z0F(height_extend=50, sensor_buffer=tolerance*2, board_bb=0);
                                    color("red")
                                    make_pololu_GP2Y0D805Z0F(
                                        height_extend=50,
                                        sensor_buffer=tolerance*2,
                                        board_bb=1,
                                        hole_offset=(i==0)?-1:0,
                                        x_extend=(i==0)?u*.5:u*2.75,
                                        x_offset=(i==0)?0:u,
                                        y_extend=u*5
                                    );
                                }
                            }
                        }
                    }
    
                    // sensor hole cutout
                    for(i=[-1:1:1]){
                        rotate([0, 0, sensor_angle*i])
                        translate([0, sensor_dist, u*2])
                        translate([(i==0)?middle_sensor_offset2:0, 0, 0])
                        //rotate([0, 90+45, 90])
                        rotate([-90-45, 0, 0])
                        rotate([0,0,(i==1)?180:0]){
                            // sensor reads "on" as long as there's something within 5cm of sensor
                            make_pololu_GP2Y0D805Z0F(height_extend=50, sensor_buffer=tolerance*2, board_bb=0);
                        }
                    }
                    
                    color("orange")
                    cylinder(d=u*24, h=height, center=true, $fn=100);
    
        
                }//end diff
                
                // side reinforcing walls
                translate([u*9-1+1/2-1, u*10 - u, u*1.5])
                cube([thickness-1,u*4/2,u*4], center=true);
                translate([-(u*9-1+1/2-1), u*10 - u, u*1.5])
                cube([thickness-1,u*4/2,u*4], center=true);
    
                difference(){
                    union(){
                        translate([0,u*11.5,0])
                        cube([u*10,u,u], center=true);
                        
                        difference(){
                            translate([0,u*11.5,u*1.5])
                            cube([u*7,u,u*4], center=true);
                            
                            translate([0,u*11.5,u*1.5+u])
                            cube([u*5,u*2,u*4], center=true);
                        }
                        
                        // middle reinforcing walls
                        translate([u*2.45,u*13,u*1.5])
                        cube([u*.1,u*4,u*4], center=true);
                        translate([-u*2.45,u*13,u*1.5])
                        cube([u*.1,u*4,u*4], center=true);
                        /*
                        rotate([0,0,30])
                        translate([u*2.45,u*13-thickness+.8,u*1.5])
                        cube([u*.1,u*4-thickness*2,u*4], center=true);
                        
                        rotate([0,0,-30])
                        translate([-u*2.45,u*13-thickness+.8,u*1.5])
                        cube([u*.1,u*4-thickness*2,u*4], center=true);
                        */
                    }
                    color("green"){
                        translate([u*3,u*11.8,0])
                        rotate([-90,0,0])
                        make_countersink();
                        translate([-u*3,u*11.8,0])
                        rotate([-90,0,0])
                        make_countersink();
                        
                        translate([u*3,u*11.8,u*3])
                        rotate([-90,0,0])
                        make_countersink();
                        translate([-u*3,u*11.8,u*3])
                        rotate([-90,0,0])
                        make_countersink();
                    }
                    
                    
                }//end diff
            }
            
            color("blue")
            cylinder(d=150 - thickness*2, h=u*50, center=true, $fn=100);
            
        }
                 
        // bottom mounting holes
        color("red"){
            for(i=[0:4])
            translate([u*4 - u*2*i, u*11.5, -u*.8])
            rotate([180,0,0])
            make_countersink(inner=u*2, outer=u);
        }
    
        // bumper mount holes
        translate([0,-1-1,u*.5])
        color("red"){
            
            translate([-u*8.8-2,u*10.5-u*.75,-u*.25])
            rotate([0,-90,0])
            make_countersink(inner=u*2, outer=u);
            translate([-u*8.8-2,u*10.5-u*.75,-u*.25+u*2.5])
            rotate([0,-90,0])
            make_countersink(inner=u*2, outer=u);
            
            mirror([1,0,0]){
                translate([-u*8.8-2,u*10.5-u*.75,-u*.25])
                rotate([0,-90,0])
                make_countersink(inner=u*2, outer=u);
                translate([-u*8.8-2,u*10.5-u*.75,-u*.25+u*2.5])
                rotate([0,-90,0])
                make_countersink(inner=u*2, outer=u);
            }
        }
        
                if(show_holes){
                    // front face mount holes
                    color("red"){
                        
                        rotate([0,0,-22.75]){
                            // left spring
                            translate([0, u*15, u*1.5])
                            rotate([-90,0,0])
                            make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                            // left contact hole
                            translate([0, u*15, -u*0])
                            rotate([-90,0,0])
                            make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                            // left wire hole top
                            translate([-u, u*14.6-u*.85+1.5/2, u*1.5])
                            rotate([0,90,0])
                            cylinder(d=1.5, h=u*2, center=true);
                            // left wire hole bottom
                            translate([-u, u*14.6-u*.85+1.5/2, u*0])
                            rotate([0,90,0])
                            cylinder(d=1.5, h=u*2, center=true);
                        }
                        
                        // right middle spring
                        rotate([0,0,7.85]){
                            // left spring
                            translate([0, u*15, 0])
                            rotate([-90,0,0])
                            make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                            // left contact hole
                            translate([0, u*15, u*1.5])
                            rotate([-90,0,0])
                            make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                            // left wire hole top
                            translate([-u, u*14.6-u*.85+1.5/2, 0])
                            rotate([0,90,0])
                            cylinder(d=1.5, h=u*2, center=true);
                            // left wire hole bottom
                            translate([-u, u*14.6-u*.85+1.5/2, u*1.5])
                            rotate([0,90,0])
                            cylinder(d=1.5, h=u*2, center=true);
                        }
                        
                        mirror([1,0,0]){
                            rotate([0,0,-22.75]){
                                // left spring
                                translate([0, u*15, u*1.5])
                                rotate([-90,0,0])
                                make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                                // left contact hole
                                translate([0, u*15, -u*0])
                                rotate([-90,0,0])
                                make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                                // left wire hole top
                                translate([-u, u*14.6-u*.85+1.5/2, u*1.5])
                                rotate([0,90,0])
                                cylinder(d=1.5, h=u*2, center=true);
                                // left wire hole bottom
                                translate([-u, u*14.6-u*.85+1.5/2, u*0])
                                rotate([0,90,0])
                                cylinder(d=1.5, h=u*2, center=true);
                            }
                            
                            // right middle spring
                            rotate([0,0,7.85]){
                                // left spring
                                translate([0, u*15, 0])
                                rotate([-90,0,0])
                                make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                                // left contact hole
                                translate([0, u*15, u*1.5])
                                rotate([-90,0,0])
                                make_countersink(inner=u*1.5, outer=u, d1=screw_thread_diameter2);
                                // left wire hole top
                                translate([-u, u*14.6-u*.85+1.5/2, 0])
                                rotate([0,90,0])
                                cylinder(d=1.5, h=u*2, center=true);
                                // left wire hole bottom
                                translate([-u, u*14.6-u*.85+1.5/2, u*1.5])
                                rotate([0,90,0])
                                cylinder(d=1.5, h=u*2, center=true);
                            }
                        }
                    }
                }
         
    }//end diff
 

    
    
}

module make_edge_sensor_mount_cover_pivot(d=0){
    //d = (d==0)?screw_head_diameter:d;
}

module make_edge_sensor_mount_cover(){
    
    height = u*4;
    thickness = 2;
    //width = u*16;//just inner box
    width = u*18;//inner box and first unit of motor mount
    
    sensor_angle = 30;
    sensor_dist = u*13;
    mount_offset_z = height/2-u/2;//-thickness/2;
    
    spring_d = 3.75;
    
    middle_sensor_offset1 = 0;
    middle_sensor_offset2 = -1;
    
    gap = 1.5;
    cover_thickness = 1;
    
    cover_d = 150-2*2;
    cover_h = u*4;
    
    mount_extension = 2;
    
    translate([0,0,0])
    difference(){
        union(){
            difference(){
                
                cylinder(d=cover_d + (gap + cover_thickness)*2, h=cover_h, center=true, $fn=100);
                
                cylinder(d=cover_d + (gap)*2, h=cover_h*2, center=true, $fn=100);
                /*
                translate([0,-1,0])
                make_edge_sensor_mount_bb(width=width+thickness-1*2, height=height, thickness=0);
                
                translate([0,-thickness,0])
                make_edge_sensor_mount_bb(width=width-1*2, height=height*2, thickness=0);
                */
                
                /*
                translate([0,u*7,0])
                cube([width*2, u*2, height*2], center=true);
                
                */
            
                // sensor hole cutout
                translate([0,0,-u*1.5])
                for(i=[-1:1:1]){
                    rotate([0, 0, sensor_angle*i])
                    translate([0, sensor_dist, u*2])
                    translate([(i==0)?middle_sensor_offset2:0, 0, 0])
                    //rotate([0, 90+45, 90])
                    rotate([-90-45, 0, 0])
                    rotate([0,0,(i==1)?180:0]){
                        // sensor reads "on" as long as there's something within 5cm of sensor
                        make_pololu_GP2Y0D805Z0F(height_extend=50, sensor_buffer=tolerance*2, board_bb=0);
                    }
                }
            
                // mid cutout
                color("orange")
                translate([0, u*6+1.5 - u*20/2, 0])
                cube([u*40, u*8 + u*20, cover_h+2], center=true);
            
                
            }//end diff

            // sidewalls
            translate([0,-u*2.5,0])
            difference(){
                color("red")
                translate([0,u*11,0])
                cube([u*17.6+2, u*4, cover_h], center=true);
                color("blue")
                translate([0,u*11,0])
                cube([u*17.6, u*8, cover_h+2], center=true);
            }
            
            difference(){
                
                // flex walls
                union(){
                    translate([u*10.5,u*10.5-0.5,0])
                    cube([u*3, 1, cover_h], center=true);
                    translate([-u*10.5,u*10.5-0.5,0])
                    cube([u*3, 1, cover_h], center=true);
                }
                
                // outside cylindrical limit
                difference(){
                    cylinder(d=cover_d + (gap + cover_thickness)*2 + u*5, h=cover_h*2, center=true, $fn=100);
                    cylinder(d=cover_d + (gap + cover_thickness)*2, h=cover_h*4, center=true, $fn=100);
                }
                
            }
            
        }   
    
        // back cutout
        color("blue")
        translate([0,u*3+5,0])
        cube([u*19, u*8, cover_h+2], center=true);
            
        // mount hole
        translate([0, -9-mount_extension, -u-u*.25])
        color("red"){
            translate([0, u*10.5+3.2, 0])                    
            rotate([0,90,0])
            cylinder(d=screw_head_diameter/2, h=u*20, center=true);
                    
            translate([0, u*10.5+3.2, u*2.5])                    
            rotate([0,90,0])
            cylinder(d=screw_head_diameter/2, h=u*20, center=true);
        }
        
    }//end diff
}
    
//translate([0,0,u/2])
make_edge_sensor_mount(show_holes=1);

//translate([0,1-1,u*1.5]) make_edge_sensor_mount_cover();
