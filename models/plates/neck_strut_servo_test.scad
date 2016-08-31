use <neck_strut_servo.scad>;

intersection(){
	rotate([0,-90,0])
	//import("../printable/neck_strut_servo_20150925.stl");
	make_neck_strut_servo2(show_servo=0, show_mounts=0);
	
	color("blue")
	translate([-52.5,0,0])
	cube([55,22,50], center=true);
}