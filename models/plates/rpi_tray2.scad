include <../settings.scad>;
use <../openscad-extra/src/countersink.scad>;
use <../openscad-extra/src/fillet.scad>;

module rpi_mount_holes2(d=screw_thread_diameter2, height=u){
    hole_d = 2.75;
    hole_sep_width = 46;
    hole_sep_depth = 55;
    x_offset = -2;
    y_offset = -7.5;
    for(i=[-1:2:1]) for(j=[-1:2:1])
    translate([(hole_sep_width + hole_d)/2*i + x_offset, (hole_sep_depth + hole_d)/2*j + y_offset, 0])
    cylinder(d=d, h=height, center=true);
}

module rpi_tray2(){
    
    corner_offset = 0;
    mount_offset_x = -1.75;
    mount_offset_y = 2;
    
    difference(){
        union(){
            // main plate
            cube([u*13 - tolerance, u*19, u*0.25], center=true);
            
            // mounting hole bulkheads
            translate([mount_offset_x, mount_offset_y, u*.5/2+u*.25/2]){
                rpi_mount_holes2(d=u, height=u*.5);
            }
            
            // mount extension
            difference(){
                color("blue")
                translate([0, (u + u*19)/2, u/2 - u*.25/2])
                cube([u*15, u, u], center=true);
                /*
                translate([0, (u + u*19)/2, u/2 - u*.25/2 + u*.5])
                cube([u*13, u*2, u], center=true);*/
            }
            
            for(i=[-1:2:1])
            translate([35*i,50,9.3])
            cube([5,5,10], center=true);
            
            color("red")
            for(i=[0:1])
            mirror([i,0,0])
            translate([27.5,50,9.35])
            rotate([0,180,0])
            make_2d_corner_fillet(width=10, height=10, depth=5);
            
            /*/ screw hole bulkheads
            color("blue")
            for(i=[0:1])
            mirror([i,0,0])
            translate([u*7.5, (u*20)/2, u/2 - u*.25/2 - u*.5])
            intersection(){
                translate([0,0,0])
                rotate([90,0,0])
                cylinder(d=u*4, h=u, center=true);
                
                translate([-u*4/2,0,u*4/2])
                cube([u*4,u*2,u*4], center=true);
                    
            }*/
            
        }
        
        color("blue")
        translate([mount_offset_x, mount_offset_y, u*.5/2+u*.25/2]){
            rpi_mount_holes2(height=u*2);
        }
        
        
        color("red"){
            translate([0, -u*19/2, 0])
            cylinder(d=u*8, h=u*2, center=true);
        }
        color("red"){
            translate([0, +u*19/2+u, 0])
            cylinder(d=u*8, h=u*2, center=true);
        }
    
        color("red")
        for(i=[-1:2:1])
        translate([u*7*i, u*10.3+1.25, u*.5- u*.25/2 + 10])
        rotate([-90,0,0])
        make_countersink(inner=10, outer=u, d1=2.5+0.5, d2=5+0.5);
        
        for(i=[0:1])
        mirror([i,0,0])
        color("green")
        translate([35+corner_offset,50+corner_offset,2-corner_offset])
        rotate([0,0,180])
        make_3d_corner_fillet(width=15, depth=15, height=15);
        
    }//end diff
}

/*
color("purple")
translate([-30, 45, 4.5])
rotate([0,0,-90])
import("../electronics/RPi2_01.stl");
*/

rpi_tray2();
