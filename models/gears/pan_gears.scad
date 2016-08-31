/*
Gears for the head/neck pan mechanism.
*/
include <../settings.scad>;
//use <parametric_involute_gear_mcad_20150830.scad>;
use <parametric_involute_gear_v5.0.scad>;

module make_pan_gears2(show_a=1, show_b=1, helpers=1){

	small_x_offset = show_a ? 64 : 0;
	small_gear_d = 8; // servo horn screw hole
	servo_horn_d = 20;//25;
	servo_horn_mount_dist = 13.7;
	servo_horn_mount_hole_d = 2.1;
	servo_horn_mount_hole_depth = 4;

	height_a = u*1;
	height_b = u*1;
	height_b_cut = 0.1;

//	cone_distance = 100;
//	teeth_small = 7;
//	teeth_large = 41;
//	outside_circular_pitch = 1000;
	outside_circular_pitch = 500-20;//adjust until teeth are aligned
	face_width = head_slew_bearing_ring_thickness;

	//outside_pitch_diameter_large  =  teeth_large * outside_circular_pitch / 180;
	outside_pitch_diameter_large = head_slew_bearing_outer_diameter-7;
   outside_pitch_diameter_small = servo_horn_d+4; //servo horn

	twist = 200;
	teeth_a = 51;
	teeth_b = 9;

	if(show_a){

		gear(
		//	number_of_teeth=41,	circular_pitch=450,
			number_of_teeth=teeth_a,	circular_pitch=360,
			gear_thickness = height_a/2,
			rim_thickness = height_a/2,
			hub_thickness = height_a/2,
			bore_diameter=head_slew_bearing_outer_diameter-head_slew_bearing_ring_thickness+1,
			twist=twist/teeth_a,
			circles=0
		);
		mirror([0,0,1])
		gear(
		//	number_of_teeth=41,	circular_pitch=450,
			number_of_teeth=teeth_a,	circular_pitch=360,
			gear_thickness = height_a/2,
			rim_thickness = height_a/2,
			hub_thickness = height_a/2,
			bore_diameter=head_slew_bearing_outer_diameter-head_slew_bearing_ring_thickness+1,
			twist=twist/teeth_a,
			circles=0
		);

	}

	if(show_b){

		translate([small_x_offset, 0, 0])
		rotate([0,0,0])
		//color("blue")
		difference(){
			union(){
				gear(
					number_of_teeth=teeth_b,	circular_pitch=360,
					gear_thickness = height_b/2,
					rim_thickness = height_b/2,
					hub_thickness = height_b/2,
					bore_diameter=0,
					twist=twist/teeth_b,
					circles=0
				);
				mirror([0,0,1])
				gear(
					number_of_teeth=teeth_b,	circular_pitch=360,
					gear_thickness = height_b/2,
					rim_thickness = height_b/2,
					hub_thickness = height_b/2,
					bore_diameter=0,
					twist=twist/teeth_b,
					circles=0
				);
			}

			// axle hole
			scale([3.5/3,3.5/3,3.5/3])
			difference(){
				cylinder(d=3, h=u*10, center=true);
				translate([1.5-(3-2.5),-5,-u*5])
				cube([10,10,u*10]);
			}


		// reduce height while keeping gear helix the same angle
		color("red"){
			translate([0,0,25/2+height_b/2-height_b_cut/2])
			cube([25,25,25],center=true);
			translate([0,0,-25/2-height_b/2+height_b_cut/2])
			cube([25,25,25],center=true);
		}
		}


	}

	if(show_a && helpers){
	
		// max footprint for large gear
		color("red")
		translate([0,0,-10])
		cylinder(d=head_slew_bearing_outer_diameter, h=1, center=true);
	
		// guide to check gear 45 degree taper
		color("red")
		translate([54,50,10])
		cylinder(d1=0, d2=servo_horn_d, h=servo_horn_d/2, center=true);

	}

	if(show_b && helpers){

		color("red")
		translate([54+33,50,10])
		cylinder(d1=0, d2=servo_horn_d, h=servo_horn_d/2, center=true);
	
		// max footprint for small gear
		color("red")
		translate([small_x_offset, 0, -10])
		cylinder(d=servo_horn_d, h=1, center=true);

	}

	//echo("teeth_large:", teeth_large);
	//echo("teeth_small:", teeth_small);
}



//make_pan_gears2(show_a=1, show_b=1, helpers=1);
//make_pan_gears2(show_a=1, show_b=0, helpers=0);//big
make_pan_gears2(show_a=0, show_b=1, helpers=0);//small
