
use <electronics/BMS706MG_Servo.scad>;

// mock up of side-bay bounding box
difference(){
    cube([50, 80, 100], center=true);
    color("red")
    translate([6, 0, 10])
    cube([60, 80-5, 100-5], center=true);
}

//scale([10, 10, 10]) import("electronics/Hitec311_Servo.stl");

translate([14, 20, -35])
rotate([0, 0, 90])
make_BMS706MG_servo(horn=0);

arm_angle_z = 0;// retracted
//arm_angle_z = 90;// 
//arm_angle_z = 180;// straight
//arm_angle_z = 180+30;// fully extended
arm_shoulder_x = -10;

translate([20, 20, 0])
//translate([-7.5, 0, 0])
//translate([arm_shoulder_x, 20, 0])
rotate([0, 0, arm_angle_z])
//translate([-arm_shoulder_x, -20, 0])
translate([-20, 0, 0])
union(){
    
    translate([arm_shoulder_x, 0, -5])
    rotate([0, 0, -90])
    scale([10, 10, 10])
    import("electronics/Servo HXT900 Micro.stl");

    translate([arm_shoulder_x, 0, 35])
    rotate([180, 0, 0])
    rotate([0, 0, -90])
    scale([10, 10, 10])
    import("electronics/Servo HXT900 Micro.stl");
        
}
