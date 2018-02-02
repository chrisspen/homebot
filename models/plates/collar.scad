include <../settings.scad>;
use <../openscad-extra/src/countersink.scad>;

module make_front_collar($fn=100){
    difference(){
        union(){
            intersection(){
                difference(){
                    cylinder(d=150, h=5*3, center=true);
                    cylinder(d=150-2*2, h=5*3+0.1, center=true);
                }
                translate([0,5*10/2 + 7.5*5+2.5,0])
                cube([27*5,5*10,3*5+1], center=true);
                
            }
            intersection(){
                // mount struts
                for(i=[-1:2:1])
                translate([5*9.5*i, 5*5/2 + 5*8, 0])
                cube([7,5*5,5*3], center=true);
                
                cylinder(d=150, h=5*3, center=true);
            }
        }
        
        color("red")
        for(i=[-1:2:1])
        translate([5*9.5*i, 55, 0])
        rotate([-90,0,0])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=20);
    }
}

module make_side_collar($fn=100){
        difference(){
        union(){
            intersection(){
                difference(){
                    cylinder(d=150, h=5*3, center=true);
                    cylinder(d=150-2*2, h=5*3+0.1, center=true);
                }
                rotate([0,0,-90])
                translate([0,5*10/2 + 5*12+2.5,0])
                cube([16*5,5*10,3*5+1], center=true);
                
            }
            
            rotate([0,0,-90])
            intersection(){
                // mount struts
                for(i=[-1:2:1])
                translate([5*5*i, 5*5/2 + 5*12.5, 0])
                cube([7, 5*5, 5*3], center=true);
                
                cylinder(d=150, h=5*3, center=true);
            }
        }
        
        color("red")
        rotate([0,0,-90])
        for(i=[-1:2:1])
        translate([5*5*i, 69, 0])
        rotate([-90,0,0])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=10);
    }
}

make_front_collar();

make_side_collar();

translate([0,0,5+2.5]){
    color("blue")
    translate([0,0,2.5])
    import("../printable/bearings_slew_bearing_outer_20160214.stl");

    color("green"){
        translate([0,0,-5-2.5])
        import("../printable/bearings_slew_bearing_outer_mount_20160215.stl");
        rotate([0,0,180])
        translate([0,0,-5-2.5])
        import("../printable/bearings_slew_bearing_outer_mount_20160215.stl");
    }
}