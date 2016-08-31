include <../settings.scad>;

module make_switch_d2f_01l83(){

    difference(){
        union(){
            color("blue")
            translate([0,0,(9.5-6.6)/2])
            cube([3.6, 9.8, 9.5], center=true);
            cube([5.75, 12.75, 6.6], center=true);
        }
        
        color("red")
        translate([0, 4.5/2+2.25/2, -6.6/2+2.25/2+0.5])
        rotate([0,90,0])
        cylinder(d=2.25, h=10, center=true);
        
        color("red")
        translate([0, -(4.5/2+2.25/2), -6.6/2+2.25/2+0.5])
        rotate([0,90,0])
        cylinder(d=2.25, h=10, center=true);
    }

}

make_switch_d2f_01l83();
