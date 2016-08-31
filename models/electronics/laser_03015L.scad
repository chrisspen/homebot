
module make_laser_03015L(buffer=0){
    difference(){
        union(){
            cylinder(d=11+buffer, h=20, center=false);
            cylinder(d=12.25, h=6.75, center=false);
        }
        translate([0,0,-1])
        cylinder(d=5, h=2, center=false);
    }
}

make_laser_03015L();
