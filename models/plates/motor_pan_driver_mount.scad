include <../settings.scad>;
use <../electronics/pololu_drv8838.scad>;
use <../openscad-extra/src/countersink.scad>;

module make_motor_pan_driver_mount(){

    difference(){
        union(){
            difference(){
                translate([0.5,0,0])
                cube([5, 15, 20], center=true);
                
                rotate([0,0,90])
                rotate([90,0,0])
                make_pololu_drv8838(tol=0.5, header_length=5);
                
                translate([0,-2,0])
                rotate([0,0,90])
                rotate([90,0,0])
                make_pololu_drv8838(tol=0.5, headers=2, header_length=5);
                
            }
            
            translate([6, 0, -6.5])
            cube([7, 15, 7], center=true);
        }
        
        color("red")
        translate([6, -7.5+5, -6.5])
        rotate([90,0,0])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=14);
    }
}

mirror([1,0,0])
rotate([-90,0,0])
make_motor_pan_driver_mount();

/*
rotate([0,0,90])
rotate([90,0,0])
make_pololu_drv8838(tol=0.5);
*/