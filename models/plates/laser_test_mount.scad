include <../settings.scad>;
use <../electronics/pi_noir_camera.scad>;
use <../electronics/laser_03015L.scad>;

module make_laser_mount_positive(thickness=u*.5, camera_offset_z=-1/2, camera_offset_y=2.75, camera_laser_dist=u*4.5){
	
        // camera bounding shape
	    translate([0,camera_offset_y,camera_offset_z]){
	        make_pi_noir_camera(buffer=0.5);
	        make_pi_noir_camera_holes(height=u*4, hole_d=2);
	    }
	    
        // laser bounding shape
	    translate([0,camera_laser_dist,6.75+thickness])
	    rotate([180,0,0])
	    make_laser_03015L(buffer=0.5-0.2);
}

module make_pi_camera_offset(){
    width = 25;
    
    beam_width = 3.75;
    
    for(i=[-1:2:1])
    translate([i*(width/2-beam_width/2), 2.5, -1.3/2])
    cube([beam_width, 1*width, 1.3], center=true);
}

module make_laser_test_mount(show_parts=0){

    thickness=u*.5;
    camera_offset_z=-1/2;
    camera_offset_y=2.75;
    camera_laser_dist=u*4.5;
    
    difference(){
    
        union(){
            // mount body
            color("red")
            translate([0,u*2,thickness/2])
            cube([u*5,u*8,thickness], center=true);
        }
    
        make_laser_mount_positive(
            thickness=thickness,
            camera_offset_z=camera_offset_z,
            camera_offset_y=camera_offset_y,
            camera_laser_dist=camera_laser_dist
        );
        
    }//end diff
    
    difference(){
        color("blue")
        make_pi_camera_offset();
    
	    translate([0,camera_offset_y,camera_offset_z-1.3])
	    make_pi_noir_camera_holes(height=u*4, hole_d=2);
    }

    if(show_parts){
	    translate([0,camera_offset_y,camera_offset_z-1.3]){
            //translate([0,0,u])
	        make_pi_noir_camera();
	        make_pi_noir_camera_holes(height=u*4, hole_d=2);
	    }
	    
	    translate([0,camera_laser_dist,6.75+thickness])
	    rotate([180,0,0])
	    make_laser_03015L();
    }

}

make_laser_test_mount(show_parts=0);

