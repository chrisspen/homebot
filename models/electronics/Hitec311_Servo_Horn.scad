module make_hitec311_horn(){
    cylinder(d=24, h=2.25);
    cylinder(d=9, h=5);
}

color("purple")
//import("electronics/Hitec311_Servo.stl");
scale([10,10,10])
//rotate([-90,0,0])
import("Hitec311_Servo.stl");

translate([0,44,0])
rotate([90,0,0])
make_hitec311_horn();