include <../settings.scad>;
use <../bearings/slew_bearing.scad>;
use <../electronics/QRD1114.scad>;
use <../plates/countersink.scad>;

rotate([0,180,0])
make_qrd1114_housing_simple(show_sensor=0, type=2);
//make_QRD1114_housing();
