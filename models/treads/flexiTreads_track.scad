include <../settings.scad>;
use <flexiTreads.scad>;
use <flexiTreads_idler.scad>;

module make_track(test_slice=0){

	trim_h = 3.5/2;

	slice_h = track_wheel_width/3;

	width_extra = 5;

	difference(){
	    track(
			r=track_radius,
			w=track_wheel_width,
			gripSize=1,
			gripWidth=track_wheel_width*0.8,
//			ws=17,//default
//			ws=10,//spaced too widely
			ws=12,//?
			teeth=track_radius
		);
	
	//bottom cutoff
		color("blue")
		translate([
			-track_radius-width_extra/2,
			-track_radius-width_extra/2,
			-track_wheel_width-trim_h*2+trim_h+(test_slice*slice_h)])
		cube([
			track_radius*2+width_extra,
			track_radius*2+width_extra,
			track_wheel_width+trim_h*2]);
	
	//top cutoff
		color("red")
		translate([
			-track_radius-width_extra/2,
			-track_radius-width_extra/2,
			track_wheel_width+trim_h*2-trim_h-(test_slice*slice_h)])
		cube([
			track_radius*2+width_extra,
			track_radius*2+width_extra,
			track_wheel_width+trim_h*2]);

	}
}

//color("red")translate([-(track_radius-track_wheel_radius-1),0,0])
make_track(test_slice=0);
//translate([0,0,0])make_track_idler();
