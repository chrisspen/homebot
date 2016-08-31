include <../settings.scad>;
use <screw_terminal.scad>;
use <../nuts.scad>;

mirror([0, 0, 0])
make_screw_terminal(
    number=4,
    rail_d=2,
    wire_d=1.5+tolerance,
    hole_d=2);
