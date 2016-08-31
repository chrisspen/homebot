use <MCAD/gridbeam.scad>;
use <MCAD/gears.scad>;//not involute
use <MCAD/involute_gears.scad>;

pi=3.1415926535897932384626433832795;

//zBeam(10);
//zBolt(10);

//demo_3d_gears();

//test_bevel_gear_pair();

/*/ bevel gears at 90 angle slightly askew
bevel_gear_pair (
    gear1_teeth = 41,
    gear2_teeth = 7,
    axis_angle = 90,
    outside_circular_pitch=1000);*/
    
//pitch = circumference = 2*pi*r
//=> r = 40 => 

pitch = 2*pi*60;
echo("pitch:", pitch);

bevel_gear_pair (
    gear1_teeth = 41,
    gear2_teeth = 7,
    axis_angle = 90,
    outside_circular_pitch=pitch//251//2*pi*40//1000-500
);

//face_width=20,//controls thickness of teeth