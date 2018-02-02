0use <plates/motor_wheel_plate_outer.scad>;
use <plates/motor_wheel_plate_inner.scad>;
use <plates/motor_wheel_wheel.scad>;
use <gears/motor_gear_drive.scad>;
use <plates/motor_wheel_axle.scad>;
use <plates/caster_housing_top.scad>;
use <openscad-extra/src/countersink.scad>;

// there's a 15mm gap between the bottom of the assembly/balast and the ground

//motor_gear_teeth = 26;

module make_skateboard_bearing(){
    //Industry Standard Size: 608 with an 8mm core, 22mm outer diameter, and 7mm width 
    difference(){
        cylinder(d=22, h=7, center=true);
        cylinder(d=8, h=7+1, center=true);
    }
}


translate([-70,0,-20-2.5-1])
rotate([0,180,0])
{
    make_caster_housing_top();
    make_caster_housing_bottom();
    make_caster_housing_center();
    translate([4,0,10])
    make_caster_wheel(d=11);
}

translate([0,0,0]){
    
    // mock support bar
    if(0)
    translate([-35-2.5,15,0])
    color("red")
    cube([5,5,20], center=true);
    
    // inner
    if(1)
    translate([0,2.5+12/2,0])make_motor_wheel_plate_inner();
    
    // there's a 12mm gap between the plates
    
    // drive gear
    if(1)
    color("orange")
    translate([-21.5,2.5,0])
    rotate([90,0,0])
    make_motor_gear_drive();//teeth=motor_gear_teeth);
    
    // mock axle
    if(0)
    color("red")
    translate([-17.5-4,1,0])
    rotate([90,0,0])
    cylinder(d=5/2, h=10, center=true, $fn=100);

    if(1)
    translate([0,-3,-15]){
        
        // mock bearing
        color("blue")translate([0,1,0])rotate([90,0,0])
        make_skateboard_bearing();
        
        color("pink")
        make_motor_wheel_wheel();//d=40, h=5, gear_teeth=motor_gear_teeth);

    }
    
    translate([0,0,-15])make_motor_wheel_axle(buffer=-0.25, hole=1);
    
    // outer
    if(1)
    translate([0,-2.5-12/2,0])make_motor_wheel_plate_outer();
    
    // mock axle
    if(0)
    color("red")
    translate([0,0,-15])
    rotate([90,0,0])
    cylinder(d=1, h=40, center=true);
}

// mock balast
if(1)
color("green")
translate([0,17,-10/2-40/2-15+15])
cube([100,10,10], center=true);

// mock clearance from bottom of balast to ground = 9mm
if(1)
color("red")
translate([0,27,-10/2-40/2-9.5])
cube([100,10,9], center=true);

// mock floor
if(1)
color("gray")
translate([0,0,-10/2-40/2-15-4])
cube([200,200,10], center=true);
