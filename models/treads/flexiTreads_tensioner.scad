include <../settings.scad>;
use <flexiTreads.scad>;
use <../nuts.scad>;

module make_track_tensioner(r=track_tensioner_radius, h=10){
	hole_d = 3;
    //hole_d = 5;
	difference(){
	    cylinder(r=r, h=h, center=true);
	    //cylinder(r=hole_d/2 + 1, h=track_wheel_gap-0.1, center=true);
	    cylinder(r=hole_d/2, h=h*2, center=true);
	}
}

module make_track_tensioner_18_10(){
    make_track_tensioner(r=18/2, h=10);
}

//make_track_tensioner();
make_track_tensioner_18_10();
