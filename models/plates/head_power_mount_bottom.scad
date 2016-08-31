include <../settings.scad>;
use <head_power_mount.scad>;

translate([0,0,u*.625])
rotate([180,0,0])
make_head_power_mount_bottom();
