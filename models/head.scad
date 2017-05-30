
include <settings.scad>;

use <bearings/slew_bearing.scad>;
use <electronics/audio.scad>;
use <electronics/camera.scad>;
use <electronics/dfrobot_rpi_arduino_hat.scad>;
use <electronics/screen.scad>;
use <electronics/slipring.scad>;
use <electronics/laser_03015L.scad>;
use <electronics/motor.scad>;
use <electronics/pololu_drv8838.scad>;
use <electronics/BMS706MG_Servo.scad>;
use <electronics/fan.scad>;
use <gears/pan_gears.scad>;
use <plates/head.scad>;
use <plates/head_bottom.scad>;
use <plates/head_face_sensor_mounts.scad>;
use <plates/head_front.scad>;
use <plates/head_power_mount.scad>;
use <plates/head_shell.scad>;
use <plates/head_top.scad>;
use <plates/motor_mount_pan.scad>;
use <plates/neck.scad>;
use <plates/rpi_tray.scad>;

module make_head_shell(){

    translate([0,0,head_slew_bearing_height+65+u/2]){
        difference(){
        
            // main spheric head mass
            union(){
                sphere(d=max_footprint, center=true);
                //translate([0,0,-max_footprint/4])
                //cylinder(h=max_footprint/2, d=max_footprint, center=true);
            }
            
            // main body internal cutout
            sphere(d=max_footprint-shell_thickness*2, center=true);
            
            // bottom cutout
            //translate([0,0,-max_footprint/2])
            //cube([max_footprint+1,max_footprint+1,max_footprint], center=true);
            
            // middle cutout
            //translate([0,max_footprint/2,max_footprint/2+shell_thickness])
            //cube([max_footprint/2,max_footprint,max_footprint], center=true);
        }
        
        difference(){
        
            // main body
            union(){
                translate([0,0,-max_footprint/4])
                cylinder(h=max_footprint/2, d=max_footprint, center=true);
            }
            
            // main body internal cutout
            //translate([0,0,-max_footprint/4])
            //cylinder(h=max_footprint/2+1, d=max_footprint-shell_thickness*2, center=true);
            
            // middle cutout
            translate([0,0,0])
            cube([head_middle_cutout,max_footprint+1,max_footprint+1], center=true);
        }
        
        translate([0,0,-max_footprint/4])
        cylinder(h=max_footprint/2, d1=max_footprint, d2=0, center=true);
        
        translate([0,0,0])
        rotate([0,90,0])
        cylinder(h=max_footprint*3, d=1, center=true);
    }
}

module head_layout(){

    rotate([0,0,0])
    translate([0,0,u/2]){
    
        translate([0,0,head_slew_bearing_height]){
            
            //color("red")
            translate([0, 0, 65]){
                
                translate([-35+u*0, 0, 0])
                rotate([-head_vertical_angle_tilt,90,180])
                //make_head_strut();
                import("printable/head_strut_servo_side.stl");
            
                translate([35-u*0, 0, 0])
                rotate([head_vertical_angle_tilt,90,0])
                //make_head_strut();
                import("printable/head_strut_open_side.stl");
                            
                color("lightgreen")
                rotate([head_vertical_angle_tilt,0,0])
                translate([0, -50+u/2, 0])
                rotate([0,0,90])
                make_head_front();
                
                color("orange")
                translate([0, 0, 0])
                rotate([head_vertical_angle_tilt,0,0])
                translate([0, 0, 30+u/2+u/2])
                rotate([0,0,0])
                //make_head_top();
                import("printable/head_top.stl");
                
                color("orange")
                translate([0, 0, 0])
                rotate([head_vertical_angle_tilt,0,0])
                translate([0, 0, 30+u/2-65-u/2])
                rotate([0,0,0])
                //make_head_top();
                import("printable/head_bottom.stl");
                
                //TODO:remove?
                translate([-1.75,0,-2.5]){
                    color("purple")
                    translate([0, 0, 0])
                    rotate([head_vertical_angle_tilt,0,0])
                    translate([-30, 50, -30+u])
                    rotate([0,0,-90])
                    import("electronics/RPi2_01.stl");
                    
                    color("MediumVioletRed")
                    rotate([head_vertical_angle_tilt,0,0])
                    translate([0,0,-7])
                    make_dfrobot_rpi_arduino_hat();
                }
                
                color("orange")
                rotate([head_vertical_angle_tilt,0,0])
                translate([0,-u*10.5,0])
                rotate([0,-90,90])
                import("printable/head_face_sensor_mount_20160214.stl");
                //make_head_face_sensor_mount();
                
                /*// top mount speaker
                rotate([head_vertical_angle_tilt,0,0])
                rotate([-50,0,0])
                translate([0,-58,0])
                rotate([0,-90,90])
                translate([0,-3,0]){
                    translate([0,-6,0])
                    make_adafruit_mono(speaker_pos=2);
                    make_speaker_mount(side=3);
                    make_speaker_mount(side=4);
                }
                */
  
                if(show_head_electronics_bb){
                    // head electronics mockup
                    translate([-100+100,-0,0])
                    color("red")
                    rotate([head_vertical_angle_tilt,0,0])
                    cube([65,100,77], center=true);//note, absolute minium x = 59
                }
                
                if(show_screen){
                    /*/screen nec
                    translate([0,-62.5,220])
                    rotate([90,0,0])
                    color("green")
                    make_35_screen();*/
                
                    //screen adafruit
                    translate([0,0,215])
                    rotate([head_vertical_angle_tilt,0,0])
                    translate([0,-62.5+5,0])
                    rotate([90,90,0])
                    color("green")
                    make_adafruit_1770_screen();
                }
                
                /*if(show_camera){
                    //make_creative_senz();
                }*/
        
                // head vertical rotation axis
                if(show_head_tilt_axis)
                color("green")
                translate([0,0,0])
                rotate([0,90,0])
                cylinder(d=2, h=200, center=true);
                        
                color("green")
                rotate([head_vertical_angle_tilt,0,0])
                translate([0, -(-u*.5-tolerance/2), 38 - 65-u])
                //rpi_tray();
                import("printable/rpi_tray2.stl");
                
            }
        
			color("brown")
            translate([0,0,42.5]){
            
                // neck left
                translate([neck_strut_spacing,0,0])
                rotate([90,0,90+180])
                //make_neck_strut();
                //import("printable/plates_plate_neck_rpi_mock_20150903.stl");
                //import("printable/neck_strut_open_20150928.stl");
                import("printable/neck_strut_open.stl");
                
                // neck right
                translate([-neck_strut_spacing,0,0])
                translate([0,0,-u*8.5])
                rotate([90,0,0])
                rotate([0,90,0])
                //make_neck_strut();
                //import("printable/plates_plate_neck_servo_mock_20150903.stl");
 				//import("printable/neck_strut_servo_20150925.stl");
 				import("printable/neck_strut_servo.stl");
                
            }
        }
        
        // photointerruptor
        translate([-50+1,0,3.5])
        rotate([180,0,0])
        import("electronics/photointerrupter_OPB940.stl");
        
        // vertical rotator servo
        translate([-70+20-10+2.75,0,20+50])
        //rotate([0,180,90])
        //scale([10,10,10])
        color("orange")
        //import("electronics/Hitec311_Servo.stl");
        //cube([2.2,2.15,4.2], center=true);
        rotate([0,-90,180])
        make_BMS706MG_servo(horn=1);
    
        // pan motor mounted downwards
        //translate([0,-.85,0])
        rotate([0,0,180]){
            mirror([0,0,0])
            rotate([0,0,25])
            translate([-60,0,30-4.5-2.5-15])
            rotate([90,0,0]){

                translate([0,-u*1.25,0]){
                    
                    color("purple")
                    translate([0,1+2.5,0])
                    make_pololu_motor_163F3F();

                }
            
                translate([0,-18,0])
                rotate([90,0,0])
                color("red")
                make_pan_gears2(show_a=0, show_b=1, helpers=0);//small

            }
            
            if(0)
            translate([-47,20,8])
            rotate([0,-90,0])
            make_pololu_drv8838();

            color("orange")
            mirror([0,0,0])
            translate([-55,-25-.75,5+2.5+.25])
            rotate([0,0,180]){
                import("printable/motor_mount_pan_top_20151224.stl");
                import("printable/motor_mount_pan_bottom_20151224.stl");
            }
        }
        /*
        color("red")
        translate([0,.65,7.25+2.5])
        rotate([0,90,0])
        cylinder(d=2.5, h=1000, center=true);
        */
        
        /*/ horizontal rotator mounted sideways
        translate([10,0,20])
        rotate([-90,-90,0]){
            scale([10,10,10])
            color("purple")
            import("electronics/Hitec311_Servo.stl");
            translate([0,44,0])
            rotate([90,0,0])
            make_hitec311_horn();
        }
       */ 
        
        /*
        make_head_bearing(
            show_outer_race = 1,
            show_inner_race_a = 1,//bottom
            show_inner_race_b = 1,//top
            show_balls = 0
        );
        */
        

        color("green")
        translate([0,0,-.4])
        rotate([0,0,-45])
        import("printable/qrd1114_mount_a_20151205a.stl");

        color("green")
        translate([0,0,-.4])
        rotate([0,0,45])
        import("printable/qrd1114_mount_a_20151205a.stl");


        //color("green")
        /*
        translate([u*2,0,0])
        rotate([180,0,0])
        translate([u*.5,0,-u*2])
        rotate([90,0,90])
        make_laser_test_mount(show_parts=0);*/
        //make_head_face_laser_mount();

        //color("blue")
        //translate([u*2.75, u*4.5, 0])
        //make_led_mount();
        
        /*

        color("blue")
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        rotate([90,0,0])
        cylinder(d=3, h=300, center=true);
        */
        
        /*
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        rotate([0,-90,0])
        import("printable/head_shell_neck_mesh_20160601.stl");
        */
        
        
        for(i=[0:1])
        rotate([0,0,180*i])
        translate([0,0,70-10+0.5])
        //rotate([head_vertical_angle_tilt,0,0])
        import("printable/head_shell_front_lower.stl");
        
        
        /*
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-u,u*6.375-65-u])
        rotate([0,180,0])
        make_head_power_mount_top();
                
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-u,u*5.75-65-u])
        rotate([0,180,0])
        make_head_power_mount_bottom();
        
        */
        
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,0,-46])
        //rotate([180,0,0])
        import("printable/head_power_mount_bottom_20151227.stl");
        
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,0,-38])
        rotate([180,0,0])
        import("printable/head_power_mount_top_20151227.stl");
        
        //// BEGIN Inner Face
        
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-47.5,0])
        //translate([0,-27.5,0])
        rotate([0,-90,-90])
        import("printable/head_face_laser_mount_20160214.stl");

        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([22.5,-61.25,0])
        //translate([0,-27.5,0])
        rotate([0,-90,-90])
        import("printable/head_face_led_mount_20160214.stl");
        
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-56.75,10])
        rotate([90,0,0])
        color("purple")
        make_rpi_camera();
        
        // rpi camera axis marker
        if(show_rpi_camera_axis)
        color("purple")
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-56.75,10])
        rotate([90,0,0])
        cylinder(d=5, h=200, center=true);
        
        color("blue")
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-69,-12.5])
        rotate([-90,0,0])
        make_laser_03015L();
        
        // laser axis marker
        if(show_laser_axis)
        color("blue")
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,-69,-12.5])
        rotate([90,0,0])
        cylinder(d=12, h=200, center=true);
        
        //// END Inner Face
        
        if(show_head_shell)
        for(i=[0:1])
        rotate([0,0,180*i])
        translate([-45+2.5,0,0])
        translate([0,0,60-2.5+12.5-87+85+2])
        rotate([0,0,180-45]){
            import("printable/head_shell_side_top.stl");
            import("printable/head_shell_side_bottom.stl");
        }
        
        /*
        color("purple")
        for(i=[0])
        rotate([0,0,180*i])
        translate([0,0,70])
        make_head_shell_side();
        */
        
        //*/
        if(show_head_shell)
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        import("printable/head_shell_top.stl");
        //*/
        
        if(show_head_shell)
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        //make_head_shell_front();
        import("printable/head_shell_front.stl");
        
        if(show_head_shell)
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        //make_head_shell_front();
        rotate([0,0,180])
        import("printable/head_shell_back.stl");
        
        if(show_head_shell)
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        for(i=[0:1])
        rotate([0,0,180*i])
        //make_head_shell_front_middle();
        import("printable/head_shell_front_middle.stl");
        
        /*
        translate([0,0,70])
        rotate([head_vertical_angle_tilt,0,0])
        //make_head_shell_back();
        import("printable/head_shell_back_20160601.stl");
        */

    }
}

/*
translate([90,0,0])
color([1,0,0,shell_alpha])
make_head_shell();
*/

head_layout();

// position1: center back
color("blue")
translate([0,0,75])
rotate([-90,0,0])
translate([0,0,71])// head radius
make_fan_F17HA_05MC();

// position1: top back
color("blue")
translate([0,0,75])
rotate([-45,0,0])
translate([0,0,71])// head radius
make_fan_F17HA_05MC();

translate([0,0,-15]){

    color("green"){
        translate([0,0,-5-2.5])
        import("printable/bearings_slew_bearing_outer_mount_20160215.stl");
        rotate([0,0,180])
        translate([0,0,-5-2.5])
        import("printable/bearings_slew_bearing_outer_mount_20160215.stl");
    }

    color("turquoise")
    translate([0,0,2.25]){
        translate([0,0,0])
        rotate([0,180,0])
        import("printable/bearings_slew_bearing_inner_top_20160214.stl");
        translate([0,0,-1])
        import("printable/bearings_slew_bearing_inner_bottom_20160214.stl");
    }

    color("gray")
    rotate([0,0,-45])
    translate([0,0,2.5])
    rotate([0,180,0])
    import("printable/qrd1114_mount_a_20151205a.stl");

    color("blue")
    translate([0,0,2.5])
    import("printable/bearings_slew_bearing_outer_20160214.stl");
    
    translate([0,0,-12])make_slipring_775();//tiniest
    
    translate([0,-53,15])
    rotate([-55,0,0])
    rotate([0,0,90])
    translate([0,0,0]){
        translate([5,0,0])
        //make_adafruit_mono(speaker_pos=2);
        //make_speaker_mount(side=3);
        import("printable/speaker_mount_a.stl");
        //make_speaker_mount(side=4);
        rotate([0,180,0])
        import("printable/speaker_mount_b.stl");
    }
    
    
        
}

color("blue")
translate([0,40,-20-2.5])
import("printable/collar_front.stl");

color("blue")
translate([70,0,-20-2.5])
import("printable/collar_side.stl");

/*
// footprint
color("gray")
translate([0,0,-100])
cylinder(d=150, h=1, center=true);
*/