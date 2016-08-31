include <../settings.scad>;
use <tray.scad>;
use <countersink.scad>;
use <../platform.scad>;
use <../torus.scad>;

module make_ballast_tray(d_offset=0, weight_boxes=1){
    
    max_height = 10;
    wall_thickness = 1;
    separation = 13*u;
    max_diameter = max_footprint + d_offset;
    box_wall_thickness = 0.5;

    difference(){
        union(){
            difference(){
                translate([0,0,max_height/2]){
                    difference(){
                        
                        // main bounding box
                        color("orange")
                        translate([0,0,max_height/2])
                        rounded_cylinder(d=max_diameter, h=max_height*2, r=max_height, center=true, $fn=100);
                    
                        // side walls cutout
                        difference(){
                        
                            translate([0,0,max_height/2 + 1])
                            rounded_cylinder(d=max_diameter-wall_thickness*2, h=max_height*2, r=max_height, center=true, $fn=100);
                            
                            color("red")
                            for(i=[-1:2:1])
                            translate([(separation-wall_thickness)*i,0,0])
                            cube([10*u, 50*u, 10*u], center=true);
                            
                        }//end diff
                        
                        color("red")
                        translate([u*0,u*10,u*10/2 + max_height/2])
                        cube([20*u, 50*u, u*10], center=true);
                        
                    }//end diff
                }
                
                color("red")
                for(i=[-1:2:1])
                translate([separation*i,0,0])
                cube([10*u, 50*u, 10*u], center=true);
            
            }//end diff
            
            for(i=[-1:2:1]){
                translate([(separation/2+u)*i,0,max_height/2])
                cube([u,u*16,max_height], center=true);
            }
        }
            
        for(i=[-1:2:1])for(j=[0:5])
        translate([(separation/2+u)*i, u*2*j - u*5, 1])
        rotate([180,0,0])
        color("blue")
        make_countersink();    
        
        translate([0,50*u/2,0])
        cube([50*u, 50*u, 50*u], center=true);
    
        // slot cutouts for wheel nuts
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([-(40-1)*i,-(u*6.5)*j,0])
        color("red")
        cube([u, u, u*20], center=true);
        
            
    }//end diff
    
    if(weight_boxes)
    color("purple")
    for(i=[0:2])
    translate([22.5*i - 22.5, -u*5.8, max_height/2+ .00])
    difference(){
        cube([19.5+box_wall_thickness*2, 57+box_wall_thickness*2, max_height-1], center=true);
        cube([19.5, 57, max_height*2], center=true);
    }
    
}

make_ballast_tray(d_offset=0, weight_boxes=1);
//make_platform();

/*
color("blue")
translate([-30,0,20])
rotate([90,0,90])
import("../printable/plates_motor_unimount_2_a_20150812.stl");
*/
