include <../settings.scad>;
use <screw_terminal.scad>;
use <../nuts.scad>;

mirror([1, 0, 0])
make_screw_terminal(number=6, rail_d=1.5+tolerance*.8, wire_d=1.5+tolerance, hole_d=2);
