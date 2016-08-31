include <../settings.scad>;
use <recharge_mount.scad>;

//translate([0,max_footprint/2-5,0])
rotate([0,0,45])
make_recharge_plug_female2(show_holes=1, show_wedge=1);

//cylinder(d=max_footprint, h=1);

//translate([0,63-2,0]) make_recharge_board_bb();

/*
translate([0,.5,-50])
import("../printable/sonar_mount_2015119.stl");
color("gray")
translate([0,-11.75+2.15,0])
cube([200,100,100], center=true);
*/