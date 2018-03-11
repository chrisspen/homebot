include <../settings.scad>;
use <dock.scad>;

//intersection(){
    union(){
        translate([-25,30,0])
        import("../printable/dock_rim_part1.stl");

        translate([-40,-45,0])
        import("../printable/dock_rim_part2.stl");

        translate([10,0,0])
        rotate([0,0,-45])
        translate([0,-85,0])
        import("../printable/dock_rim_part3.stl");

        translate([40,-75,0])
        import("../printable/dock_rim_part4.stl");

        translate([25,30,0])
        import("../printable/dock_rim_part5.stl");
    }

    //cube([150,150,150],center=true);
//}

// max bed area on printrbot
if(0)
color("gray")
cube([150,150,1],center=true);
