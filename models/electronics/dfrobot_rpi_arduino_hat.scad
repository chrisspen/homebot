
include <../settings.scad>;

module make_dfrobot_rpi_arduino_hat(){

    translate([0,0,0])
    cube([61,85,1.65], center=true);
    
    translate([0,0,8.4/2+1.65/2])
    cube([58,81,8.4], center=true);

}

make_dfrobot_rpi_arduino_hat();
