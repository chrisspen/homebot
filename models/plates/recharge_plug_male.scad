include <../settings.scad>;
use <recharge_mount.scad>;

translate([0,u*4.5,0])
rotate([180,0,0])
make_recharge_plug_male(half=+1);
make_recharge_plug_male(half=-1);
