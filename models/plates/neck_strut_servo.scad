use <neck.scad>;

rotate([0,0,-90])
rotate([0,-90,0])
make_neck_strut_servo2(show_holes=1);
//make_neck_strut_servo(holes=0);

/*
difference(){
    make_neck_strut_servo_mount();

        // vertical rotator servo
        translate([-10,0,65-(43/2-9.3)])
        color("orange")
        rotate([0,-90,180])
        make_BMS706MG_servo_holes(d=screw_thread_diameter);
}
*/