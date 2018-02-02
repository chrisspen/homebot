include <settings.scad>;

        if(show_motor_inner_plates){
            color("red")
            translate([
                horizontal_plate_width/2+unit_thickness*1.5,
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90+180])
            //make_motor_unimount_2(do_countersink=0);
            //make_motor_unimount_2_a();
            import("printable/motor_unimount_2_a0_20160406.stl");

            color("red")
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
            color("red")
            translate([
                horizontal_plate_width/2+unit_thickness*1.5 + u + track_wheel_gap,
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90+180])
            //make_motor_unimount_2_b();
            import("printable/motor_unimount_2_b0_20160406.stl");

            color("red")
            translate([
                -(horizontal_plate_width/2+unit_thickness*1.5 + u + track_wheel_gap),
                0,
                bottom_clearance+motor_plate_offset_z
            ])
            rotate([90,0,90])
            //make_motor_unimount_2_b();
            import("printable/motor_unimount_2_b0_20160406.stl");
        }