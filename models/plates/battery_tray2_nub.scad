use <battery_tray2.scad>;

difference(){

    make_battery_nub();

    translate([50,50,0])
    cube([100,100,100],center=true);

}

//color("gray") cube([80,80,1],center=true);