module make_motor_wheel_axle(buffer=0, hole=1){
    // gap is 12mm, with 2.5mm inset on each side to a 8mm hole
    length = 12+2.5+2.5;
    rotate([90,0,0]){
        difference(){
            intersection(){
                cube([7.5+buffer,7.5+buffer,length], center=true);
                cylinder(d=8+buffer, h=length+1, center=true, $fn=100);
            }
            //translate([2,0,0])cube([7,1,length+1], center=true);
            if(hole)
            cylinder(d=2.5, h=50, center=true, $fn=100);
        }
    }
}

module make_motor_wheel_axle_cutout(){
    make_motor_wheel_axle(buffer=0.5, hole=0);
}

make_motor_wheel_axle(buffer=-0.25, hole=1);
