include <../settings.scad>;
use <../openscad-extra/src/countersink.scad>;
use <arduino_tray.scad>;

module make_head_arduino_mount(){
    difference(){
        union(){
            // main mouting surface
            cube([55,60,2.5], center=true);
            
            // hanging rim
            translate([55/2-5/2,0,-5/2-2.5/2])
            cube([5,60,5+1], center=true);
        }
        
        // hanging rim holes
        color("red")
        for(i=[-1:2:1])
        translate([55/2,30/2*i,-5/2-2.5/2-1/2])
        rotate([0,90,0])
        make_countersink(d1=2.5+0.5);
        
        // arduino mount holes
        color("blue")
        translate([0,-7,0])
        make_arduino_holes(d=2);
            
        // mass savers
        color("blue"){
            cylinder(d=12, h=10, center=true);
            for(i=[0:5])
            rotate([0,0,60*i])
            translate([15,0,0])
            cylinder(d=12, h=10, center=true);
        }
    }
    
}

module make_head_arduino_mount_spacer(){
    difference(){
        cylinder(d=5, h=2, center=true);
        cylinder(d=2.5, h=2*2, center=true);
    }
}

rotate([0,180,0]){

    if(0)
    color("gray")
    translate([0,-7,5])
    rotate([90,0,90])
    scale([10,10,10])
    import("../electronics/Arduino_UNO.stl");

    make_head_arduino_mount();

    color("green")
    for(i=[0:3])
    translate([i*10-10,-70/2,0.25])
    make_head_arduino_mount_spacer();

}