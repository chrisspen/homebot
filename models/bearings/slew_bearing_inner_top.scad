include <../settings.scad>;
use <slew_bearing.scad>;
use <../electronics/QRD1114.scad>;
use <../plates/countersink.scad>;

rotate([0,180,0])
make_head_bearing(
    show_outer_race = 0,
    show_inner_race_a = 1,
    show_inner_race_b = 0,
    show_balls = 0,
    show_holes=1,
    show_gears=1,
    gap=0.5,
    show_body_mount=0
);
