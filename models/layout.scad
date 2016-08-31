include <settings.scad>;

use <wheel.scad>;
use <electronics/battery.scad>;
use <torso.scad>;
use <head.scad>;
use <casters/caster.scad>;
use <plates/plate.scad>;
use <plates/vertical.scad>;
use <plates/motor.scad>;
use <plates/neck.scad>;
use <plates/head.scad>;
use <plates/head_bottom.scad>;
use <plates/head_top.scad>;
use <plates/head_front.scad>;
use <electronics/Maestro6.scad>;
use <electronics/slipring.scad>;
use <electronics/converter.scad>;
use <electronics/screen.scad>;
use <electronics/camera.scad>;
use <electronics/QRD1114.scad>;
use <electronics/Hitec311_Servo_Horn.scad>;
use <electronics/motor.scad>;
use <electronics/circuit_breakers.scad>;
use <bearings/bearing.scad>;
use <bearings/slew_bearing.scad>;
use <gears/motor_gear.scad>;
use <gears/pan_gears.scad>;
//use <treads/trackCustom.scad>;
use <plates/motor_unimount_2.scad>;
use <plates/motor_unimount_2_a.scad>;
use <plates/motor_unimount_2_b.scad>;
use <plates/vertical_battery_slotted.scad>;
use <plates/vertical_computer.scad>;
use <plates/vertical_back_slotted.scad>;
use <plates/tray.scad>;
use <plates/battery_tray.scad>;
use <plates/dc_converter_tray.scad>;
use <plates/edge_sensor_mount.scad>;
use <plates/u_beam.scad>;
use <treads/flexiTreads_idler.scad>;
use <treads/flexiTreads_driver.scad>;
use <bearings/slew_bearing_outer_mount.scad>;
use <plates/conduit_beam.scad>;
use <plates/recharge_mount.scad>;
use <plates/screw_terminal.scad>;
use <electronics/pololu_pushbutton.scad>;
use <plates/side_frame_extension.scad>;
use <plates/sonar_mount.scad>;
use <plates/power_switch_mount.scad>;
use <electronics/pololu_pushbutton_1439.scad>;
use <plates/top_front_panel.scad>;
use <plates/lower_rear_panel.scad>;
use <platform.scad>;
use <plates/ballast_tray.scad>;
use <plates/top_rear_panel.scad>;
use <plates/side_panel.scad>;

color("red")
rotate([0,0,180]){

    if(show_platform){
        make_platform();
    }

    if(show_motor){
        color("green"){

        translate([-motor_offset_x,motor_offset_y,motor_offset_z+bottom_clearance])
        make_motor(flip=1);
        
        translate([motor_offset_x,-motor_offset_y,motor_offset_z+bottom_clearance])
        make_motor();

        }
    }

    if(show_wheel){
        color("red"){

        translate([wheel_offset_x,0,wheel_offset_z])
        make_wheel();
        
        translate([-wheel_offset_x,0,wheel_offset_z])
        make_wheel();

        }
    }

    if(show_torso){
        color([0,0,1,shell_alpha])
        translate([0,0,torso_height/2+bottom_clearance])
        make_torso_shell();
    }

    if(show_head){
        rotate([0,0,180])
        translate([0,0,bottom_clearance-u*3.5]){
            translate([0,0,torso_height+head_offset_z+u*2]){

                translate([0,0,-head_offset_z+u*1])
                head_layout();

                color("brown"){
                    
                    for(i=[0:1])
                    mirror([i,0,0])
                    translate([0,0,-62.5-u*3])
                    //make_neck_mount_side();
                    //import("printable/slew_bearing_outer_mount_double_20151129.stl");
                    import("printable/bearings_slew_bearing_outer_mount_20160215.stl");
                    /*
                    translate([0,0,-62.5-u])
                    //make_neck_mount_side();
                    import("printable/slew_bearing_outer_mount_20151121.stl");
                    
                    mirror([1,0,0])
                    translate([0,0,-62.5-u])
                    //make_neck_mount_side();
                    import("printable/slew_bearing_outer_mount_20151121.stl");
                    
                    mirror([0,1,0])
                    translate([0,0,-62.5-u])
                    //make_neck_mount_side();
                    import("printable/slew_bearing_outer_mount_20151121.stl");
                    
                    mirror([0,1,0])
                    mirror([1,0,0])
                    translate([0,0,-62.5-u])
                    //make_neck_mount_side();
                    import("printable/slew_bearing_outer_mount_20151121.stl");
                    */
                    
                    
                }

                if(show_head_shell){
                    difference(){
                        color([0,0,1,shell_alpha])
                        make_head_shell();
            
                        translate([-100,0,0])
                        cube([200,200,200], center=true);
                      }
                }
            }

        }

    }



    for(i=[0:1])
    rotate([0,0,90*i])
    translate([-head_slew_bearing_inner_diameter/2+2+1, 0, bottom_clearance+125+u*0.25+u])
    rotate([-90,0,90])
    color("green")
    make_QRD1114();

    if(show_caster){
        translate([0,caster_offset_y,caster_diameter/2])
        make_caster();
        translate([0,-caster_offset_y,caster_diameter/2])
        make_caster();
    }

    if(show_battery){

        /*
        rotate([0,0,90])
        translate([-12.5,0,battery_height/2+bottom_clearance+battery_offset_z+6])
        //make_battery();
        //	make_battery_tray_1();
        make_battery_dc12300();
        //make_battery_dc1298A();//too big
        */

        translate([
            u*7.5,
            0,
            vertical_battery_plate_height/2 + motor_plate_height + bottom_clearance - u/2
        ])
        rotate([90,0,90])
        //make_vertical_battery_slotted();
        import("printable/vertical_battery_slotted_20150915.stl");

        translate([
            -u*7.5,
            0,
            vertical_battery_plate_height/2 + motor_plate_height + bottom_clearance - u/2
        ])
        rotate([90,0,90+180])
        //make_vertical_battery_slotted();
        import("printable/vertical_battery_slotted_20150915.stl");

        translate([
            0,
            u*7.5,
            vertical_battery_plate_height/2 + motor_plate_height + bottom_clearance - u/2
        ])
        rotate([90,0,180])
        color("orange")
        make_vertical_back_slotted();

        translate([
            0,
            0,
            bottom_clearance + u*7
        ])
        rotate([0,0,180])
        import("printable/battery_tray2_cartridge_bottom_20160522.stl");
        
        translate([
            0,
            0,
            bottom_clearance + u*10.5
        ])
        rotate([0,180,0])
        rotate([0,0,180])
        import("printable/battery_tray2_cartridge_top_20160522.stl");
    }


        translate([0,-50-2,battery_height/2+bottom_clearance+battery_offset_z-24])
        rotate([90,0,0])
        color("red")
        make_5a_step_down_converter();
        color("green")
        translate([0,-u*8.25,bottom_clearance+u*3.5])
        rotate([90,0,0])
        //make_dc_converter_tray();
        import("printable/dc_converter_tray_20150917.stl");

    if(show_rpi){

    /*
        translate([0,-13,u*0.5]){
            // RPi hat mockup
            color("blue")
            translate([0,-40+53,79-5+bottom_clearance+30])
            cube([56, 65, 10], center=true);

            color("green")
            translate([0,-40+53,79-5+bottom_clearance+20])
            cube([56, 65, 10], center=true);
            color("blue")
            translate([0,-40+53,79-5+bottom_clearance+10])
            cube([56, 65, 10], center=true);
            color("green")
            translate([0,-40+53,79-5+bottom_clearance])
            cube([56, 65, 10], center=true);
            
            color("orange")
            translate([28,-40,60+bottom_clearance])
            rotate([0,0,90])
            import("electronics/RPi2_01.stl");

            //battery tray
            translate([0,-u*0.0,bottom_clearance+u*11+0.1])
            color("green")
            make_plate_tray(width=70, depth=95);
        }
    */

        // arduino motor controller mockup
        color("blue")
        translate([-6,0,bottom_clearance+90+4])
        cube([54,76,56], center=true);

    /*
        translate([u*7.5,0,u*23.5])
        rotate([90,0,-90])
        make_vertical_computer_frame();*/
    }

    if(show_foot_cube){
        color([0,1,0,0.5])
        translate([0,0,304.8/2])
        cube([304.8,304.8,304.8], center=true);
    }

    if(show_horizontal_plates){
        translate([0,0,bottom_clearance])
        //make_bottom_plate();
        //make_horizontal_frame();
        import("printable/horizontal_20150726.stl");

        translate([0,0,bottom_clearance+u*7])
        //make_bottom_plate();
        //make_horizontal_frame();
        //import("printable/horizontal_20150726.stl");
        import("printable/horizontal_motor_top_slotted_20150915.stl");
    }

    if(show_caster_plates){
        translate([0, -vertical_plate_width/2-unit_thickness/2, bottom_clearance+unit_thickness*4])
        rotate([0,180,0])
        import("printable/plate_caster_20150727.stl");
        
        translate([0, -(-vertical_plate_width/2-unit_thickness/2), bottom_clearance+unit_thickness*4])
        rotate([0,180,180])
        import("printable/plate_caster_20150727.stl");
    }

    if(show_vertical_plates){
        translate([-(horizontal_plate_width/2+unit_thickness/2), 0, bottom_clearance+vertical_plate_height/2-unit_thickness/2])
        rotate([90,0,90+180])
        //make_vertical_frame();
        import("printable/vertical_slotted_20150915.stl");
        
        translate([+(horizontal_plate_width/2+unit_thickness/2), 0, bottom_clearance+vertical_plate_height/2-unit_thickness/2])
        rotate([90,0,90])
        //make_vertical_frame();
        import("printable/vertical_slotted_20150915.stl");

    }

    //translate([vertical_plate_offset_x,0,bottom_clearance+vertical_plate_offset_z])
    //make_vertical_motor_plate();

    if(show_tracks){


        //color("red")
        //translate([track_offset_x,0,track_offset_z])
        //scale([1,1,1])
        //trackV1("trackPlate.dxf");

    }

    if(show_track_wheels){

        echo("wheel(lr).y:", -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y);
        echo("wheel(lr).z:", -wheel_hub_offset/2+bottom_clearance);

        // left 1
            translate([
                horizontal_plate_width/2+u*2 + track_wheel_width_buffer,
                -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
                -wheel_hub_offset/2+bottom_clearance
            ])
        rotate([0,90,0])
        make_track_idler();
        // left 2
            translate([
                -(horizontal_plate_width/2+u*2 + track_wheel_width_buffer +u*2),
                -(motor_plate_width/2+wheel_hub_offset/2)-motor_arm_offset_y,
                -wheel_hub_offset/2+bottom_clearance
            ])
        rotate([0,90,0])
        make_track_idler();

        // right 1
            translate([
            horizontal_plate_width/2+u*2 + track_wheel_width_buffer,
                +(motor_plate_width/2+wheel_hub_offset/2+motor_arm_offset_y),
                -wheel_hub_offset/2+bottom_clearance
            ])
        rotate([0,90,0])
        make_track_idler();
        // right 2
            translate([
            -(horizontal_plate_width/2+u*2 + track_wheel_width_buffer +u*2),
                +(motor_plate_width/2+wheel_hub_offset/2+motor_arm_offset_y),
                -wheel_hub_offset/2+bottom_clearance
            ])
        rotate([0,90,0])
        make_track_idler();

        echo("wheel(c).y:", -motor_offset_y);
        echo("wheel(c).z:", bottom_clearance + motor_plate_height/2 + motor_hole_height_offset/2);

        // center 1
            translate([
                horizontal_plate_width/2+u*2 + track_wheel_width_buffer,
                -motor_offset_y,
                bottom_clearance + motor_plate_height/2 + motor_hole_height_offset/2
            ])
        rotate([0,90,0])
        make_track_driver();
        // center 2
            translate([
                -(horizontal_plate_width/2+u*2 + track_wheel_width_buffer +u*2),
                motor_offset_y,
                bottom_clearance + motor_plate_height/2 + motor_hole_height_offset/2
            ])
        rotate([0,90,0])
        make_track_driver();
    }

    if(show_sonar_mount){
        translate([0,0,bottom_clearance+u*4 + u*8.5]){
            
            color("brown")
            translate([0,0,0])
            //make_sonar_mount();
            import("printable/sonar_mount_2015119.stl");
            
            color("red")
            for(i=[-1:1:1])
            rotate([0,0,30*i])
            translate([0,u*12+2,0])
            rotate([-90,0,0])
            import("electronics/SEN136B5B_Ultrasonic_Sensor.stl");

        }
    }

    if(show_recharge_mount){
        translate([0,0,bottom_clearance+u*5.5]){
            color("red")
            //make_recharge_mount();
            //import("printable/recharge_mount_20151114.stl");
            color("green")
            //translate([0, 69, 0]) import("printable/recharge_plug_female_20151116.stl");
            translate([0, 69.5, 0]) rotate([0,0,-45]) import("printable/recharge_plug_female_v2_20160519.stl");
        }
    }

    if(show_conduit){
        translate([0,0,bottom_clearance]){
            color("purple")
            translate([0,u*9.5,24*u/2])
            rotate([90,0,180])
            //make_conduit_beam(x_units=3, z_units=25, holes=1);
            import("printable/conduit_beam_20151108.stl");
        }
    }

    if(show_edge_mount){

        translate([0,0,bottom_clearance]){

        color("blue")
        translate([0,0,-u*0])
        //make_edge_sensor_mount();
        import("printable/edge_sensor_mount_20151110.stl");
            
        color("orange")
        translate([0,0,u*1.5])
        rotate([0,180,0])
        import("printable/edge_sensor_mount_cover_20151111b.stl");
    /*
        color("purple")
        translate([0,0,bottom_clearance+u*4])
        make_edge_sensor_mount();
    */
        }
    }

    if(show_motor_plates){
        
        if(show_motor_inner_plates){
            translate([
                horizontal_plate_width/2+unit_thickness*1.5,
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90+180])
            //make_motor_unimount_2(do_countersink=0);
            //make_motor_unimount_2_a();
            import("printable/motor_unimount_2_a0_20160406.stl");

            translate([
                -(horizontal_plate_width/2+unit_thickness*1.5),
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90])
            //make_motor_unimount_2(do_countersink=0);
            //make_motor_unimount_2_a();
            import("printable/motor_unimount_2_a0_20160406.stl");
        }
        
        if(show_motor_outer_plates){
            translate([
                horizontal_plate_width/2+unit_thickness*1.5 + u + track_wheel_gap,
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90+180])
            //make_motor_unimount_2_b();
            import("printable/motor_unimount_2_b0_20160406.stl");

            translate([
                -(horizontal_plate_width/2+unit_thickness*1.5 + u + track_wheel_gap),
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90])
            //make_motor_unimount_2_b();
            import("printable/motor_unimount_2_b0_20160406.stl");
        }
    }

    if(show_torso_shell){
        
        color("orange")
        translate([0, -u*0, bottom_clearance + u*3.5])
        rotate([0, 0, 180])
        import("printable/lower_rear_panel_20151122.stl");

        color("lightgreen")
        translate([0, -u*0, bottom_clearance + u*16])
        rotate([0,0,180])
        //make_top_rear_panel(show_door=0);
        import("printable/top_rear_panel_20151122.stl");

        for(i=[0:1])
        color("pink")
        rotate([0,0,180*i])
        translate([-u*0, -u*0, bottom_clearance + u*12])
        rotate([0,0,90])
        //make_side_panel();
        import("printable/side_panel_20151204b.stl");
        
        for(i=[0:1])
        mirror([1*i,0,0])
        translate([0,0,75])
        rotate([0,180,0])
        rotate([180,0,0])
        import("printable/battery_tray2_nub_20160522.stl");
        
    }

    color("purple")
    translate([-u*0, -u*2.5, u*18])
    //import("printable/ballast_tray_20160329.stl");
    import("printable/arduino_tray_2_20160508.stl");

    translate([-u*0, -u*0, u*2 - u*0])
    //import("printable/ballast_tray_20160329.stl");
    import("printable/ballast_tray_20160414.stl");

    translate([-u*0, -u*0, u*2 - u*0])
    rotate([0,0,180])
    import("printable/ballast_tray_20160414.stl");

    /*
    // aligner
    translate([-u*8.5, -u*5, bottom_clearance + u*23.5])
    rotate([90,0,0])
    cylinder(d=2, h=u*100, center=true);
    */

    /*
    translate([0, u*11.5, bottom_clearance + u*20.5])
    rotate([90,0,0])
    make_pololu_pushbutton_1439();
    */

    /*
    color("purple")
    translate([0, -u*8.5, bottom_clearance + u*24])
    rotate([0,180,0])
    rotate([0,0,90])3
    make_power_switch_mount(show_switch=1);
    */

    color("brown")
    //translate([u*2, -u*13, bottom_clearance + u*10.5 - u])
    translate([-u*8, -u*8.5, bottom_clearance + u*18])
    mirror([1,0,0])
    rotate([0,90,0])
    rotate([0,90,-90])
    //make_screw_terminal();
    import("printable/screw_terminal_4_20160524.stl");

    for(j=[-1:2:1]) for(i=[-1:2:1])
    color("orange")
    translate([(u*10.25)*i, -u*7.5*j, bottom_clearance + u*16])
    rotate([90, 0, 180*((i+1)/2)])
    //make_side_frame_extension();
    import("printable/side_frame_extension_20151129.stl");

    translate([0, u*0, bottom_clearance + u*21]){
        //make_top_front_panel();
        import("printable/top_front_panel_20151122.stl");

        //translate([0,0,.5])
        for(i=[0:1])
        mirror([1*i,0,0])
        import("printable/top_front_panel_door_20151122.stl");
    }

    if(show_recharge_dock)
    translate([0, u*35, u*.5])
    rotate([0,0,0])
    make_recharge_dock();

    // circuit breaker
    translate([0, -u*10, u*30])
    //rotate([0,0,90])
    //make_w58_series();
    //make_ts_701();
    //make_104_pr();
    rotate([90,0,0]) make_1410();

    translate([0,0,120-5/2])
    rotate([0,0,180])
    import("printable/top_rear_panel2_20160526.stl");

    color("green")
    translate([0,0,145+1.5])
    rotate([180,0,0])
    import("printable/torso_wire_restrainer.stl");

}

/*
translate([0,0,10])
import("printable/lrf_calibration_centerpoint.stl");

for(j=[0:1])
mirror([0, 0, 1*j]) translate([0,0,-15*j])
for(i=[0:1])
mirror([1*i, 0, 0])
translate([42.5,0,5])
rotate([0,180,0])
import("printable/lrf_calibration_nub.stl");
*/
