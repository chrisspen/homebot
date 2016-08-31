include <../settings.scad>;

module make_servo_bearing(od=0, id=0, h=0){
    // od = outer diameter
    // id = inner diameter
    // h = height or thickness
    od = od ? od : motor_bearing_outer_diameter;
    id = id ? id : motor_bearing_inner_diameter;
    h = h ? h : motor_bearing_thickness;
    difference(){
        cylinder(d=od, h=h, center=true);
        cylinder(d=id, h=h+1, center=true);
    }
}

make_servo_bearing();
