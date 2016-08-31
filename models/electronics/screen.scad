include <../settings.scad>;

module make_35_screen(){
    //http://www.neosecsolutions.com//products.php?47&cPath=20
    cube([83,55,20], center=true);
}
module make_adafruit_1770_screen(){
    //https://www.adafruit.com/products/1770
    /*
    Board: 51mm x 81mm x 2mm / 2" x 3.2" x 0.08"
    Screen: 50mm x 69mm x 4mm / 2" x 2.7" x 0.2"
    Mounting Holes: 54mm x 74mm / 2.1" x 2.9"
    */
	hole_od = 3;

	color("red")
    cube([51,81,2], center=true);//board

	color("blue")
	translate([0,0,2+1])
    cube([50,69,4], center=true);//screen

	color("green"){
	    cube([54,74,1.5], center=true);//holes

		hull(){
			for(i=[-1:2:1]){
				translate([54/2*i,74/2+hole_od/2,0])
				cylinder(d=hole_od, h=1.5, center=true);
			}
		}
		hull(){
			for(i=[-1:2:1]){
				translate([54/2*i,-(74/2+hole_od/2),0])
				cylinder(d=hole_od, h=1.5, center=true);
			}
		}
	}
}

//make_35_screen();
make_adafruit_1770_screen();
