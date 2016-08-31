include <settings.scad>;

module make_wheel(){
    rotate([0,90,0])
    cylinder(h=10, d=wheel_diameter, center=true);
}

make_wheel();
