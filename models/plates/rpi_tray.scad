include <../settings.scad>;
use <countersink.scad>;

module rpi_mount_holes(d=screw_thread_diameter2, height=u){
    hole_d = 2.75;
    hole_sep_width = 46;
    hole_sep_depth = 55;
    x_offset = -2;
    y_offset = -7.5;
    for(i=[-1:2:1]) for(j=[-1:2:1])
    translate([(hole_sep_width + hole_d)/2*i + x_offset, (hole_sep_depth + hole_d)/2*j + y_offset, 0])
    cylinder(d=d, h=height, center=true);
}

module rpi_tray(){
    
    difference(){
        union(){
            // main plate
            cube([u*13 - tolerance, u*19, u*0.25], center=true);
            
            // mounting hole bulkheads
            translate([0, 0, u*.5/2+u*.25/2]){
                rpi_mount_holes(d=u, height=u*.5);
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
            
        }
        
        color("red"){
            translate([0, -u*19/2, 0])
            cylinder(d=u*8, h=u, center=true);
        }
        
        color("blue")
        translate([0, 0, u*.5/2+u*.25/2]){
            rpi_mount_holes(height=u*2);
        }
        
        for(i=[-1:2:1])
        translate([u*7*i, u*10.3, u*.5- u*.25/2])
        rotate([-90,0,0])
        make_countersink(inner=u*2, outer=u);
        
    }//end diff
}

/*
color("purple")
translate([-30, 45, 4.5])
rotate([0,0,-90])
import("../electronics/RPi2_01.stl");
*/

rpi_tray();
