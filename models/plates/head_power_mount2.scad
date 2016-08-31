include <../settings.scad>;
use <countersink.scad>;
use <screw_terminal.scad>;

module ubec_5a_bb(width=u*6, depth=u*2.5){
    
    h2 = u-u*.25;
    
    // main body
    translate([0,0,u*1])
    cube([depth, width, u*2], center=true);
    
    /*/ cutout for wires
    translate([0,0,-h2/2])
    cube([u, width+u*2, h2], center=true);
    */
    
    // cutout for wire clampdown
    translate([0,0,-h2/2])
    cube([u*2.5, width+u*2, h2], center=true);
    
    // cross section for wires
    for(i=[-1:2:1]){
        translate([0, (width+u*2.75)/2*i, -h2/2])
        cube([u*4, u*2, h2], center=true);
        
        translate([0, (width+u*2.75)/2*i, u*2])
        cube([u*2, u, u*4], center=true);
    }
}

module head_power_mount_holes(){
    for(i=[-1:2:1]) for(j=[-1:2:1])
    translate([u*6*i-u*0, u*5*j, u*.33-u*.5])
    rotate([0,0,0])
    make_countersink(d1=screw_thread_diameter+tolerance/2);
}

module head_power_mount_holes2(){
    color("brown")
    for(i=[-1:2:1]) for(j=[-1:2:1])
    translate([u*6*i, u*6*j, u*.33])
    rotate([0,0,0])
    cylinder(d=2, h=u*2, center=true);
}

module make_head_power_mount_top(){
    
    ubec_width = u*11.25;
    
    depth = u*9;
    
    ubec_depth = u*2.5;
    
    ubec_sep = (depth - ubec_depth*3)/3;
    
    hole_offset = u*3;
    
    hole_position = [u*3, -u*0, u*0];

    difference(){
        union(){
            difference(){
                union(){
                    color("green")
                    translate([0,u*0,0])
                    cube([u*13, u*20, u*.25], center=true);
                    
                    for(i=[-1:2:1])
                    color("blue")
                    translate([-u*6*i, 0, u*.5 - u*.25/2])
                    cube([u, u*20, u], center=true);
            
                }
            
                color("purple")
                translate([0,0,u])
                head_power_mount_holes();
                
                translate(hole_position)
                translate([0,0,u])make_header_pin_housing_peg_holes(hole_offset=hole_offset);
            
                translate([0,0,0])
                head_power_mount_holes2();
                
            }// end diff
            
            difference(){
                
                // ubec cutout mass
                color("orange")
                translate([0, ubec_width/2 - (ubec_width - u*10), u*.5 - u*.25/2])
                cube([u*11-u, ubec_width, u], center=true);

                // ubec cutout bb
                for(i=[0:3])
                color("red")
                rotate([0,0,90])
                translate([u*2.75*i +u*.25, u*0, u*.8775])
                ubec_5a_bb(depth=ubec_depth);
                
            }// end diff
            
            translate(hole_position)       
            make_header_pin_housing_pegs(hole_offset=hole_offset);
        }

        translate(hole_position)
        translate([0,0,u])make_header_pin_housing_peg_holes(hole_offset=hole_offset);
    
    }// end diff    
}

module make_head_power_mount_bottom(){
    
    depth = u*9;
    
    ubec_depth = u*2.5;
    
    ubec_sep = (depth - ubec_depth*3)/3;
    
    difference(){
        cube([u*13, u*20, u*.25], center=true);
    
        // ubec cutout bb
        for(i=[0:3])
        color("red")
        rotate([0,0,90])
        translate([u*2.75*i +u*.25, u*0, u*.8775-u*2])
        ubec_5a_bb(depth=ubec_depth);
        
        color("blue")
        translate([u-u, -u*5.75, 0])
        cube([u*7,u*8,u], center=true);
    
        // large wire hole gap
        color("blue")
        translate([-u*4.5, -u*8, 0])
        cube([2.5, u*3, u], center=true);
    
        // large wire straing gauge
        color("blue")
        translate([-u*4.5, -u*10, 0])
        cube([1.6, u*2, u], center=true);
        
        head_power_mount_holes2();
    
        color("purple"){
            head_power_mount_holes();
        }
        
    }//end diff
}

module make_header_bb(){
    rotate([0,90,0])
    cube([5.1, 2.6*5, 14], center=true);
}

module make_header_pin_housing_peg_body(back_depth, hole_offset, height, tol, d=0){
    d = (d==0) ? 3.5+tol : d;
    for(i=[-1:2:1])
    translate([-back_depth/2, hole_offset*i, -height/2])
    cylinder(d=d, h=height, center=true);
}

module make_header_pin_housing_peg_holes(hole_offset=u*1.85){
    back_depth = 2.5;
    height_ext = 3;
    height = 5.1+tolerance;
    //hole_offset = u*1.85;
    
    //difference(){
        translate([0,0,0])
        make_header_pin_housing_peg_body(back_depth, hole_offset, height*2, tol=0, d=2);
}
    
module make_header_pin_housing_pegs(hole_offset=u*1.85){
    back_depth = 2.5;
    height_ext = 3;
    height = 5.1+tolerance;
    
    //difference(){
        translate([0,0,height/2])
        make_header_pin_housing_peg_body(back_depth, hole_offset, height/2, tol=0);
      /*  
        for(i=[-1:2:1])
        translate([-back_depth/2, hole_offset*i, 0])
        cylinder(d=2, h=u*2, center=true);
    }
    */
    
}

module make_header_pin_housing(hole_offset=u*1.85){
    
    back_depth = 2.5;
    height_ext = 3;
    height = 5.1+tolerance;
    base_hole_offset = u*1.85;
    extra_length = (hole_offset-base_hole_offset)+u*2;
    slot_offset = -u*.75;
    
    difference(){
    
        // main body
        color("purple")
        translate([-back_depth/2,0,1/2])
        cube([2.5+tolerance+1+back_depth, 13+tolerance+u*2 + extra_length, height+1], center=true);
        
        // slots
        for(i=[0:7])
        color("red")
        translate([0, 2.54*i - 13/2 + 2.54/2 + slot_offset, -height_ext/2])
        cube([u*1, .75, 5.1+tolerance+height_ext], center=true);
        
        // interior hollow
        color("red")
        translate([-back_depth/2,0,-height_ext/2])
        cube([2.5+tolerance+back_depth, 13+tolerance+extra_length/2, 5.1+tolerance+height_ext], center=true);
        
        for(i=[-1:2:1])
        translate([-back_depth/2, hole_offset*i, 0])
        cylinder(d=2, h=u*2, center=true);
    
        /*
        for(i=[-1:2:1])
        translate([-back_depth/2, hole_offset*i, -height/2])
        cylinder(d=3.5+tolerance, h=height, center=true);
        */
        make_header_pin_housing_peg_body(back_depth, hole_offset, height, tolerance);
        
    }
    
    
}

module make_head_power_terminal(offset=u*2.5){
    /*
    translate([offset,0,u*.6])
    make_screw_terminal(
        number=5,
        show_tabs=0,
        rail_d=1.5+tolerance*.8,
        wire_d=1.5+tolerance,
        hole_d=2
    );
    */
    
    /*
    color("green")
    translate([u*3,0,u*.6])
    make_header_bb();

    translate([u*0,0,u*.6 + u*2])
    make_header_pin_housing();
    */
    
    hole_offset=u*1.85;
    
    difference(){
        union(){
            color("blue")
            cube([u*13, u*5, u*.25], center=true);
        
            for(i=[-1:2:1])
            translate([u*6*i, 0, u*.625])
            cube([u, u*5, u], center=true);
            
            make_header_pin_housing_pegs(hole_offset=hole_offset);
        }
        
        color("purple")
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([u*6*i, u*j, u-1])
        rotate([0,0,0])
        make_countersink(d1=screw_thread_diameter+tolerance/2);
        
        translate([0,0,u])make_header_pin_housing_peg_holes(hole_offset=hole_offset);
    }
    
}

//color("blue")
make_head_power_mount_top();

translate([0+u*2.5, -u*0, u*.7])
//rotate([0,-90,0])
rotate([0,0,180])
make_header_pin_housing(hole_offset=u*3);

//make_head_power_terminal();

//translate([0,0,u*1])rotate([0,0,0])make_head_power_mount_bottom();

//translate([0,u*7,0])make_head_power_terminal();


/*
*/
color("purple")
translate([0, 0, 0])
translate([0, 0, -u*.625])
rotate([0,90,0])
//make_head_top();
import("../printable/head_top_20151001.stl");
