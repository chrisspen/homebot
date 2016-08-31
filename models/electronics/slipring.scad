include <../settings.scad>;

module make_slipring_736(){
    //https://www.adafruit.com/products/736
    //6 wiries
	total_height_with_wires = 32;
	flange_d = 44;
	hole_d = 5.5;
	difference(){
	    union(){
			//top wires
			translate([0,0,-total_height_with_wires+9+2])
			cylinder(h=total_height_with_wires, d=4, center=false);
			//top thin
			cylinder(h=9, d=8, center=false);
			//middle flange
			translate([0,0,-3])
			cylinder(h=3, d=flange_d, center=false);
			//bottom fat
			translate([0,0,-19.5])
			cylinder(d=22, h=19.5, center=false);
		}
		for(i=[0:2]){
			rotate([0,0,120*i])
			translate([0,flange_d/2-hole_d/2-1.8,0])
			cylinder(d=hole_d, h=50, center=true);
		}
	}
}

module make_slipring_775(){
    //https://www.adafruit.com/products/775
    //6 wiries
    cylinder(h=17, d=12);
}

module make_slipring_1196(){
    //https://www.adafruit.com/products/1196
    total_height_with_wires = 32;
    flange_d = 44;
    hole_d = 5.5;
    difference(){
        union(){
            //top wires
            translate([0,0,-total_height_with_wires+9+2])
            cylinder(h=total_height_with_wires, d=4, center=false);
            //top thin
            cylinder(h=9, d=8, center=false);
            //middle flange
            translate([0,0,-3])
            cylinder(h=3, d=flange_d, center=false);
            //bottom fat
            translate([0,0,-19.5])
            cylinder(d=22, h=19.5, center=false);
        }
        for(i=[0:2]){
            rotate([0,0,120*i])
            translate([0,flange_d/2-hole_d/2-1.8,0])
            cylinder(d=hole_d, h=50, center=true);
        }
    }
}

make_slipring_736();
