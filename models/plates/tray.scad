include <../settings.scad>;
use <plate.scad>;


// This supports the frame vertically and attaches to the horizontal support.
module make_plate_tray(width, depth, thickness=1, height=0, front=1, back=1, rail_width=u*0.5){
    
    height = height ? height : u;
    rail_cutout_height = screw_head_height+0.15;
//    d1 = screw_thread_diameter;
//    d2 = screw_head_diameter+0.1;
    d1 = 1;
    d2 = 1;
    rail_cutout_height = u*0.25;

    difference(){
        union(){
            translate([-width/2,-depth/2,0])
            cube([width,depth,thickness], center=false);
        
            // front rail
            if(front){
                color("red")
                translate([0, -(depth/2-rail_width/2), height/2])
                cube([width,rail_width,height], center=true);
            }
            
            // back rail
            if(back){
                color("red")
                translate([0, depth/2-rail_width/2, height/2])
                cube([width, rail_width, height], center=true);
            }
            
            // left rail
            color("blue")
            translate([-width/2+rail_width/2,0,height/2])
            cube([rail_width,depth,height], center=true);
            
            // right rail
            color("blue")
            translate([width/2-rail_width/2,0,height/2])
            cube([rail_width,depth,height], center=true);
        }
        /*
        translate([width/2-u*2,depth/2,u/2])
        rotate([90,0,0])
        cylinder(d=screw_thread_diameter+tolerance, h=u*2, center=true);
        
        translate([-(width/2-u*2),depth/2,u/2])
        rotate([90,0,0])
        cylinder(d=screw_thread_diameter+tolerance, h=u*2, center=true);
        */

    }//end diff
}

make_plate_tray(width=70, depth=75, rail_width=1);
