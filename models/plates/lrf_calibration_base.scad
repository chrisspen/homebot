use <lrf_calibration.scad>;

d = 75;

make_lrf_prop_base(d=d);

translate([45/100*d, 45/100*d, 0])
make_lrf_prop_base_block();
