include <../settings.scad>;
use <../gears/motor_gear_drive.scad>;
use <../gears/parametric_involute_gear_v5.0.scad>;

module make_motor_wheel_wheel(d=40+7, h=5, gear_teeth=motor_gear_teeth){
    circular_pitch = 180+1;
    height_b = 5;    
    twist = 200;
    teeth = 45;
    
    difference(){
        union(){
            rotate([90,0,0])
            union(){
                //cylinder(d=d, h=h, center=true, $fn=100);
                
                // gap filler between wheel and gear
                translate([0,0,-0.5])cylinder(d=28, h=h, center=true, $fn=100);
            }
            
            // wheel gear
            color("blue")
            translate([0,5.5,0])
            rotate([0,-6.5,0])
            rotate([-90,0,0])
            make_motor_gear_base(teeth=gear_teeth);
            
            // outer rim with gear tread
            color("red")
            translate([0,0,0])
            rotate([90,0,0])
            for(i=[0:1:1])
            mirror([0,0,i])
            gear(
                number_of_teeth=teeth,
                circular_pitch=circular_pitch,
                gear_thickness = height_b/2,
                rim_thickness = height_b/2,
                hub_thickness = height_b/2,
                bore_diameter=0,
                twist=twist/teeth,
                circles=0
            );
        }

        // cutout for 608 skateboard bearing
        translate([0,1+0.25/2-0.01,0])
        rotate([90,0,0])
        cylinder(d=22+0.25, h=7+0.25, center=true, $fn=100);
        
        // cutout for 8mm axle with buffer
        rotate([90,0,0])
        cylinder(d=12, h=100, center=true, $fn=100);
    
        // cutout for 608 skateboard bearing top space
        translate([0,1.5,0])
        rotate([90,0,0])
        cylinder(d=19, h=8, center=true, $fn=100);
    
    }
        
}

rotate([90,0,0])
make_motor_wheel_wheel();
