use <../openscad-extra/src/countersink.scad>;
use <motor_wheel_axle.scad>;

module make_motor_wheel_plate_outer(){
    difference(){
        union(){
            cube([80, 5, 40], center=true);
            
            //bottom curve to support wheel axle
            translate([0,0,-15])
            rotate([90,0,0])
            cylinder(d=25, h=5, center=true, $fn=100);
        }
        
        // Axle cutout
        //translate([0,(12+5)/2,-15])rotate([90,0,0])cylinder(d=8+0.5, h=12+5, center=true, $fn=100);
        translate([0,(12+5)/2,-15])rotate([0,0,0])make_motor_wheel_axle_cutout();
        
        // Axle set screw hole
        color("red")translate([0,-2.6,-15])rotate([90,0,0])make_countersink(d1=2.5, d2=5, inner=20);

        // outer plate attachment screw holes
        for(i=[-1:2:1])
        color("red")
        translate([(80/2-10-10)*i,-2.6,20-2.5])
        rotate([90,0,0])
        make_countersink(d1=2.5+0.5, d2=5, inner=50);
        
        // shell mounting holes
        for(i=[-1:2:1])
        color("red")
        translate([(80/2-10)*i,0,-40/2+10])
        rotate([90,0,0])
        cylinder(d=2.5, h=50, center=true, $fn=100);

    }// end diff
}

rotate([0,0,90])
rotate([90,0,0])
make_motor_wheel_plate_outer();
