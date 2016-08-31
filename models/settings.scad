// Common settings used by all scad files.

// ALL UNITS IN MILLIMETERS (MM)!!!

$fn = 30;//low-res but fast to render
//$fn = 50;//higher-res but slow to render

pi = 3.1415926535897932384626433832795;

max_footprint = 150; // 15x15 cm

bottom_clearance = 10;
bottom_clearance = 25.4/2+10;
//bottom_clearance = 30;

shell_alpha = 0.215;
//shell_alpha = 1.0;

tolerance = 0.5;

// Flat countersunk M2.5mm x 0.45mm machine screw.
screw_thread_diameter = 2.5;//2.8;//2.75;//2.4?
screw_thread_length = 5;//?
screw_head_diameter = 4.75; // M2.5
//screw_head_diameter = 5.2; // M3
screw_head_height = 1.5;//2;

screw_thread_diameter2 = 2;

unit_thickness = 5;
u = unit_thickness;

shell_thickness = unit_thickness;

// Show flags.
show_platform = 0;
show_head = 1;
show_head_shell = 0;
show_head_electronics_bb = 0;
show_battery = 1;
show_foot_cube = 0;
show_torso = 0;
show_motor = 0;
show_motor_plates = 1;
show_motor_inner_plates = 1;
show_motor_outer_plates = 1;
show_wheel = 0;//obsolete
show_tracks = 1;//
show_track_wheels = 1;
show_caster = 0;
show_rpi = 1;
show_vertical_plates = 1;//
show_horizontal_plates = 1;//
show_caster_plates = 0;//obsolete?
show_camera = 1;
show_screen = 0;
show_edge_mount = 1;
show_sonar_mount = 1;
show_recharge_mount = 1;
show_conduit = 1;
show_torso_shell = 1;
show_recharge_dock = 0;
show_head_tilt_axis = 0;
show_rpi_camera_axis = 0;
show_laser_axis = 0;

head_vertical_angle_tilt = 0;
//head_vertical_angle_tilt = 30;//max
//head_vertical_angle_tilt = 10;

torso_height = 120+20;

base_plate_diameter = max_footprint-unit_thickness*2;
base_plate_frame = 0;

head_height = 50;
head_offset_z = max_footprint/2;
head_middle_cutout = 85;

rpi_with_four_hats_and_clearance = 76;

qrd1114_w = 6.10;
qrd1114_l = 4.39;
qrd1114_h = 4.65;

dock_radius = 10;
dock_gripper_border = 2;
dock_gripper_instep = 0.9;
dock_gripper_cutout_tapper = 50;
dock_receiver_height = 5;

// Patriod Fuel+ 9000 mAh
battery_length = 90;
battery_width = 57;
battery_height = 23;
battery_offset_z = 30;//4;

wheel_diameter = 50;
wheel_offset_x = 65;
wheel_offset_z = wheel_diameter/2;
wheel_axle_diameter = 5;

//caster_diameter = 19.05; // 0.75 inches
caster_diameter = 25.45; // 1 inches
//caster_diameter = 15.875; // 5/8 inches
caster_holder_diameter = 30;//31;
caster_offset_y = base_plate_diameter/2 - caster_holder_diameter/2;

arduino_offset_x = 0;
arduino_offset_y = 0;
arduino_offset_z = 67;
arduino_offset_x2 = 0;
arduino_offset_y2 = -20;
arduino_offset_z2 = torso_height+bottom_clearance;

horizontal_plate_width = unit_thickness*8*2 - unit_thickness*2;
horizontal_plate_depth = unit_thickness*8*2;

vertical_plate_width = horizontal_plate_depth;
vertical_plate_height = unit_thickness*8;
vertical_plate_offset_x = wheel_offset_x-22.5;
vertical_plate_offset_z = vertical_plate_height/2 + unit_thickness/2;

vertical_battery_plate_height = 100 - 15;

caster_plate_width = horizontal_plate_width + unit_thickness*2;
caster_plate_mount_length = unit_thickness*4;
caster_plate_width_minor = caster_holder_diameter;

motor_switch_length = 20;
motor_switch_thickness = 6.5;
motor_switch_height = 11;
motor_switch_height_leads = 23;
motor_switch_on_height =  13.3;
motor_switch_hole_diameter = 2.5;
motor_switch_hole_offset_z = 6.5;//this plus diameter/2 is hole center from switch top
motor_switch_hole_spacing = 7.1;// holes are diameter/2 from this

motor_length = 60;
motor_width = 25;
motor_collar_width = 7;
motor_collar_height = 2.5;
motor_axle_width = 4;
motor_axle_height = 9.5;
motor_offset_x = 10;
//motor_offset_y = motor_width/2+5;
motor_offset_y = unit_thickness*3.5;
motor_offset_z = vertical_plate_height/2 - unit_thickness/2;//4;//30;
motor_mount_holes_dist = 17;
// motor countersink hole
motor_mount_holes_width = 3; // M3
motor_mount_holes_head_height = 2.2;
motor_mount_holes_head_width = 5.8;
motor_bearing_outer_diameter = 10;
motor_bearing_inner_diameter = 5.95;
motor_bearing_thickness = 2.5;

motor_unimount_idler_axle_d = screw_thread_diameter;
//motor_unimount_spacer_depth = 0.1;//too small
motor_unimount_spacer_depth = 0.5;

motor_arm_offset_y = -u*3;
motor_arm_offset_z = u*.5;

motor_plate_width = vertical_plate_width;
motor_plate_height = vertical_plate_height;
motor_plate_thickness = unit_thickness;
motor_plate_offset_z = vertical_plate_offset_z;

motor_plate_middle_thickness = 5;

wheel_hub_offset = u*3;

motor_hole_height_offset = -5;

nylon_spacer_outer_diameter = 6.35;
nylon_spacer_inner_diameter = 3.302;
nylon_spacer_length = 12.7;

track_width = unit_thickness*3;
track_offset_x = -50;
track_offset_z = .1;
//track_wheel_radius = 19;//original
track_wheel_radius = 14;
//track_wheel_hole_radius = (nylon_spacer_outer_diameter+1)/2;
track_wheel_hole_radius = motor_bearing_outer_diameter/2;

track_driver_hole_radius = (motor_axle_width)/2 + 0.1;
track_driver_set_screw_head_d = 4.1;
track_driver_set_screw_shaft_d = 2.4;
//track_wheel_width = 13.5;//original
track_wheel_width = u*2;
track_wheel_width_buffer = u*0.25;
track_offset = 3;        //distance from chassis
track_wheel_offset = track_offset;
//track_length = 250; // see tread_triangle.py, too big
//track_length = 240; // see tread_triangle.py
//track_length = 238; //inc/dec in 10 unit increments so notches match idlers
track_length = 228; //inc/dec in 10 unit increments so notches match idlers
track_radius = track_length/2/pi;
track_wheel_show_fancy = 1;
track_wheel_gap = track_wheel_width + track_wheel_width_buffer*2;
track_axle_bearing_diameter = motor_bearing_inner_diameter - 0.1;
track_tensioner_radius = 12;
track_nut_height = 3.5; //TODO
track_punchout_thickness = 0.25;//should be the same as the minimum layer thickness

head_slew_bearing_outer_diameter = 115;
head_slew_bearing_ring_thickness = 20;
head_slew_bearing_inner_diameter = head_slew_bearing_outer_diameter - head_slew_bearing_ring_thickness;
//head_slew_bearing_hole_diameter = head_slew_bearing_inner_diameter - head_slew_bearing_ring_thickness;
head_slew_bearing_hole_diameter = 12+tolerance; // slip ring https://www.adafruit.com/products/775
//head_slew_bearing_ball_diameter = 5;
head_slew_bearing_ball_diameter = 2.5; //u/2;
head_slew_bearing_height = u;//head_slew_bearing_ball_diameter*2;
head_slew_bearing_gap = 0.5;
head_slew_bearing_slew_gap = 0;
head_slew_bearing_gap_factor = 0.1;

neck_strut_width = u*7;
neck_strut_height = u*18;
neck_strut_spacing = u*8;

show_outer_race = 0;
show_inner_race_a = 1;
show_inner_race_b = 1;
show_balls = 0;
