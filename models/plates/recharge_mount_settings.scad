include <../settings.scad>;

recharge_mount_width = u*8;//-u*.5; //u*26;//inner box and first unit of motor mount
recharge_mount_height = u*4;//-u*.5;
recharge_mount_thickness = 2;

recharge_mount_terminal_offset = u*2;
recharge_mount_terminal_extend = 4.5;

recharge_mount_slot_mag = 1;

// radius of rounded cube used for the indentation
recharge_plug_box_r = 2;

recharge_plug_female_box_tol_xz = 0; // subtraction of width and height
recharge_plug_female_box_tol_y = 1.5; // offset in depth

//recharge_plug_male_box_tol_xz = 2.5; // subtraction of width and height
recharge_plug_male_box_tol_xz = 4.5; // subtraction of width and height
recharge_plug_male_box_tol_y = 1.5; // offset in depth

recharge_plug_terminal_extension = 0;

recharge_plug_wire_d = 2.75;

recharge_dock_width = u*20;
