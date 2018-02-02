include <../settings.scad>;
use <battery_tray2.scad>;
use <../openscad-extra/src/countersink.scad>;

module make_head_torso_jack(){
    
    difference(){
    
        intersection(){
            // main mass
            color("red")
            translate([0, 60-1.25, -5])
            cube([12*5, 12.5, 5], center=true);
        
            cylinder(d=150-18, h=20, center=true, $fn=100);
        }
    
        // top/back cutout
        translate([0,58,-5])
        scale([1.05,1.05,1])
        rotate([180,0,0])
        make_blade_connector_male(bb=1);
        
        // wire holes
        for(i=[-1:2:1])
        translate([20*i, 52.5, 0])
        scale([1,2,1])
        cylinder(d=5, h=50, center=true, $fn=100);
    
        // screw holes
        color("green")
        for(i=[-1:2:1])
        translate([(6.5*5-7.5)*i, 60, -5])
        rotate([-90,0,0])
        make_countersink(d1=2.5+0.5, inner=20);
        
    }
    
}
    
module make_torso_head_jack(){

    difference(){
        union(){
            
            intersection(){
                    
                // main mass
                color("purple")
                translate([0,55+1.5-9,-12.5])
                cube([14*5, 7*5, 2*5], center=true);
                    
                cylinder(d=150-18, h=100, center=true, $fn=100);
                
            }
            
            // mount studs
            color("red")
            for(i=[-1:2:1]) for(j=[0:1])
            translate([32.5*i, 55 -3*5*j - 5-2.5, -20])
            cube([5, 5, 5], center=true);
        }
        
        // outer cutouts
        color("blue")
        for(i=[-1:2:1])
        translate([26*i, 62.5-2.5, -12])
        cube([22, 20, 15], center=true);
    
        // upper cutouts
        color("blue")
        for(i=[-1:2:1])
        translate([26*i, 50-6, -5])
        cube([22, 30, 15], center=true);    
        
        /*/ inner cutout
        color("blue")
        translate([0,35+2.5,-15])
        cube([12*5, 10, 20], center=true);*/
        
        // top hole cutout
        translate([0,57,-12.5])
        rotate([90,0,0])
        make_blade_connector_female(bb=1, extra_x=0, extra_y=10, extra_z=0, extra_x2=0, extra_y2=0);
        
        // back hole cutout
        translate([0,57,-12.5])
        scale([1.015,1,1.04])
        rotate([90,0,0])
        make_blade_connector_female(bb=1, extra_x=0, extra_y=0, extra_z=0, extra_x2=0, extra_y2=0, extra_z2=25);
        
        // screw holes
        for(i=[-1:2:1])
        translate([6.5*5*i, 50, -20])
        rotate([-90,0,0])
        make_countersink(d1=2.5+0.5, inner=23);
        
    }// end diff
    
}

if(0)
translate([0,58,-5])
rotate([180,0,0])
make_blade_connector_male();

if(0)
translate([0,57,-12.5])
rotate([90,0,0])
make_blade_connector_female(bb=1, extra_x=0, extra_y=0, extra_z=0, extra_x2=0, extra_y2=0, extra_z2=0);

make_head_torso_jack();

make_torso_head_jack();

color("orange"){
    
    if(1)
    import("../printable/bearings_slew_bearing_outer_20160214.stl");

    for(i=[0:1])
    mirror([1*i,0,0])
    translate([0,0,-10])
    import("../printable/bearings_slew_bearing_outer_mount_20160215.stl");

    if(0)
    translate([0,60,-5-2.5+5])
    import("../printable/head_shell_back_lower.stl");

}

color("green")
translate([0,50-5-2.5,-20])
rotate([90,0,0])
rotate([0,0,90])
import("../printable/power_switch_mount_20151120.stl");

if(0)
color("gray")
translate([0,0,-50])
cylinder(d=max_footprint, h=1, center=true, $fn=50);

