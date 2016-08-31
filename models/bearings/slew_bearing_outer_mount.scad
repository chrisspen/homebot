include <../settings.scad>;
use <slew_bearing.scad>;
use <../plates/countersink.scad>;

module make_neck_mount_side(show_torso=0, show_countersinks=1, $fn=100){

	difference(){
		// main body
		difference(){
			//color("blue")
            union(){
                        
                // side tabs
                translate([-u*10.25, -u*4.5+u*0, -u])
                cube([u*4.5, u*5+u*0.5+u*1.5, u*3], center=true);
                
                // top tab
                translate([-u*11, -u*4, u*1])cube([u, u*8, u], center=true);
                            
                // bottom horizontal bulk head
                translate([-u*10.25, -u*4, -u*2])
                cube([u*4.5, u*6, u], center=true);
                
            }

            // extra mounting holes
            translate([-u*10.25, -u*4, -u*2])
            color("red"){
                for(i=[0:2])
                translate([0,u-u*i,0])
                rotate([0,90,0])
                cylinder(d=screw_thread_diameter, h=u*10, center=true);
            }
            
			// cutout for inner edge
			cylinder(d=head_slew_bearing_outer_diameter-u*2, h=u*3, center=true, $fn=$fn);
            
			//make_bearing_horizontal_mount_holes(d_offset=11, as_countersinks=1, itotal=1, iadd=1);

			if(show_countersinks){

				// smaller/middle mount countersink
				color("red")
				translate([-u*7.8-u-u*5,-u*7.5+u*1.5,-u*2])
				rotate([0,-90,0])
				make_countersink();

				// longer/outer mount countersink
				color("red")
				translate([-u*7.8-u*2.5-u*1.8, -u*7.5+u*1.5+u*4, -u*2])
				rotate([0,-90,0])
				make_countersink();

			}

		}//end diff
          
        for(z=[0:2]){
            
            // extra side mounting holes
            for(i=[0:5])
            color("red")
            translate([-u*12-u, -u*7.5+u*1.5+u*i-u, -u*1+u*z])
            rotate([0,-90,0])
            make_countersink(outer=u*10, inner=u*15);
            
            if(z<2){
            // extra front mounting holes
            for(i=[0:3]) for(j=[0:1])
            color("red")
            translate([-u*12+u*i+u*.5,-u*8.5,-u*1-u*j+u*z])
            rotate([0,-90,90])
            make_countersink(outer=0, inner=u*3);
            }
        }

        // top mount countersinks
        color("red")
        translate([-u*11.3, -u*5, u])
        rotate([0,-90,0])
        make_countersink(outer=u*2, inner=u*5);
        
	}// end diff
    
	if(show_torso){
		// mock torso
		color("green")
		translate([0,0,-u*9.5])
		cube([u*16,u*16,u*16], center=true);
	}
    
}

module make_nect_mount_side_double(){
    difference(){
        union(){
            make_neck_mount_side(show_countersinks=1, $fn=100);
            
            mirror([0,1,0])make_neck_mount_side(show_countersinks=1, $fn=100);
            
            translate([-u*10.25,0,-u*2])
            cube([u*4.5, u*2, u], center=true);
        }
        
        color("red")
        for(i=[0:2])
        translate([-u*13, u-u*i, -u*2])
        rotate([0,-90,0])
        make_countersink(outer=0, inner=u*5);
        
        // cutout to help joints mesh
        for(i=[-1:2:1])
        color("blue")
        translate([-u*10.25, u*7.5*i, -u*2])
        cube([u*6, u+tolerance, u+tolerance], center=true);
        
    }
}

make_nect_mount_side_double();
