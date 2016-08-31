include <../settings.scad>;
use <neck_strut_servo.scad>;
use <../electronics/BMS706MG_Servo.scad>;

rotate([0,90,0])
difference(){
	make_neck_strut_servo_mount(n=1);

        // vertical rotator servo holes
        translate([-10,0,65-(43/2-9.3)])
        color("orange")
        rotate([0,-90,180])
        make_BMS706MG_servo_holes(d=screw_thread_diameter);
}