include <../settings.scad>;

module make_rpi_camera(){
    //http://uk.rs-online.com/web/p/video-modules/7757731/
	translate([0,0,2.5+2]){
    cylinder(d=8.5, h=5, center=true);
    translate([0,0,-2-2.5])
    cube([25,20,4], center=true);
	}
}

module make_creative_senz(){
    //
    cube([4.27*25.4, 2.03*25.4, 2.11*25.4], center=true);
}

//make_rpi_camera();
make_creative_senz();