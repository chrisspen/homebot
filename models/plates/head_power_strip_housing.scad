include <../settings.scad>;
use <head_power_mount.scad>;

rotate([0,180,0])
//make_header_pin_housing();
make_header_pin_housing(hole_offset=u*3);
