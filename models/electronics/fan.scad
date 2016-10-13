// http://www.digikey.com/product-detail/en/copal-electronics-inc/F17HA-05MC/563-1108-ND/1165502
module make_fan_F17HA_05MC(){
    difference(){
        cube([17,17,8], center=true);
        color("red")
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([13.5/2*i,13.5/2*j,0])
        cylinder(d=2, h=10, center=true, $fn=100);
    }
}

make_fan_F17HA_05MC();
