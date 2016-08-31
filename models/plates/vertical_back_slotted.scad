include <../settings.scad>;
use <plate.scad>;
use <cross_plate.scad>;
use <countersink.scad>;

module _make_horizontal_beam(width, skip_i=-1){
    make_generic_beam(
        u=u,
        a=width/u-2,
        b=width/u,
        d=screw_thread_diameter,
        end_fills=0.5,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=0,//outward countersinks
        cy2=0,
		skip_i=skip_i
    );
}

// This supports the frame vertically and attaches to the horizontal support.
module make_vertical_back_slotted(){
    width = vertical_plate_width-u*2;
    height = vertical_battery_plate_height;
    d = screw_thread_diameter;
    
    //top
    translate([0,height/2-u/2,0])
    rotate([0,0,0])
    _make_horizontal_beam(width);
    
    //middle
    translate([0,-height/2+u*(6.5-2),0]){
    rotate([0,0,0])
//    _make_horizontal_beam(width, skip_i=1);
    make_generic_beam(
        u=u,
        a=width/u-2,
        b=width/u,
        d=screw_thread_diameter,
        end_fills=0.5,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=1,//outward countersinks
        cy2=0,
		skip_i=1
    );
	cube([width/2,u,u], center=true);
	}

	/*/bottom big holes
	translate([0,0,u*.3])
	color("blue"){
		translate([-u*6,-u*8,0])
		make_countersink(d1=screw_thread_diameter+tolerance);
		translate([+u*6,-u*8,0])
		make_countersink(d1=screw_thread_diameter+tolerance);
		translate([-u*6,-u*4,0])
		make_countersink(d1=screw_thread_diameter+tolerance);
		translate([+u*6,-u*4,0])
		make_countersink(d1=screw_thread_diameter+tolerance);
	}*/

    //bottom
    translate([0,-(height/2-u/2),0]){
	    rotate([0,0,0])
	//    _make_horizontal_beam(width, skip_i=1);
	    make_generic_beam(
	        u=u,
	        a=width/u-2,
	        b=width/u,
	        d=screw_thread_diameter,
	        end_fills=0.5,
	        countersink_d=screw_head_diameter,
	        countersink_d2=screw_thread_diameter,
	        countersink_depth=screw_head_height,
	        cx1=0,
	        cx2=0,
	        cy1=1,//outward countersinks
	        cy2=0,
			skip_i=1
	    );
		cube([width/2,u,u], center=true);

		difference(){
			translate([0,-u,0])
		    make_generic_beam(
		        u=u,
		        a=width/u-2,
		        b=width/u,
		        d=screw_thread_diameter,
		        end_fills=0.5,
		        countersink_d=screw_head_diameter,
		        countersink_d2=screw_thread_diameter,
		        countersink_depth=screw_head_height,
		        cx1=0,
		        cx2=0,
		        cy1=1,//outward countersinks
		        cy2=0,
				skip_i=1
		    );
			color("red")
			translate([0,-u,-u/2+tolerance/2])
			cube([width-u*2,u,u], center=true);
		}
	}

    //left
    translate([(width/2-u/2),0,0])
    rotate([0,90,0])
    rotate([0,0,90])
    make_generic_beam(
        u=u,
        a=height/u,
        d=d,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=0,
        cy2=0//outward countersinks
    );

    //right
    translate([-(width/2-u/2),0,0])
    rotate([0,90,0])
    rotate([0,0,90])
    make_generic_beam(
        u=u,
        a=height/u,
        d=d,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=0,
        cy2=0//outward countersinks
    );
    
	rotate([0,90,90])
	make_cross_beams_1(w=width, h=height, t=u);

	// fill in extra partial holes
	translate([width/2-u/2,-(height/2-u),0])
	cube([u,u,u], center=true);
	translate([-(width/2-u/2),-(height/2-u),0])
	cube([u,u,u], center=true);

}

rotate([0,180,0])
make_vertical_back_slotted();
