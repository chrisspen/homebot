include <../settings.scad>;
use <plate.scad>;
use <countersink.scad>;
use <vertical.scad>;
use <../electronics/motor.scad>;

module make_motor_mount_pan_pad(full=0, h=u*4, length=u*5.5){
	
	translate([0,3 + (h-25)/2, -2]){
	if(full){
		color("red")
		translate([-u*2-u/2-u*5/2, 0, u*3.5])
		rotate([0,90,0])
		cube([length*2, h*2, u*5], center=true);
	}else{
		translate([-u*2,0,u*3.5+1.0])
		rotate([0,90,0])
		cube([length, h, u], center=true);
	}
	}
}

module make_motor_mount_pan(half=0){

	h = u*4;//26.25;
	arm_length = u*5;

	difference(){

		union(){
			rotate([0,25,0])
			difference(){
				union(){
					// motor cowling
					hull(){
						color("green")
						translate([0,3 + (h-25)/2,0])
						rotate([90,0,0])
						cylinder(d=18, h=h, center=true, $fn=50);
					
						color("blue")
						translate([-15, 3 + (h-25)/2,0])
						rotate([90,0,0])
						cylinder(d=18, h=h, center=true, $fn=50);
					}
		
					// mount arm
					translate([0,3 + (h-25)/2,0])
					translate([-arm_length/2,0,0])
					cube([arm_length, h, u], center=true);

				}
			
				// space for motor	
				translate([0,0,0])
				scale([1.05,1,1.05])//10.5/10
				make_pololu_motor_163F3F();
			
				// gap for motor wire leads
				//translate([0,18,0])
				//cube([15,u*2,5], center=true);
		
				rotate([90,0,0])
				cylinder(d=4.25, h=100, center=true);
		
				if(half){
					translate([0,0,-50/2])
					cube([50,50,50], center=true);
				}
		
				//make_motor_mount_pan_pad(full=1, h=h);
		
				/*/ pull tie cutout
				color("red"){
					translate([0,2.75/2,0])
					difference(){
						translate([0,0,0])
						rotate([90,0,0])
						cylinder(d=15, h=2.75, center=true);
						translate([0,0,0])
						rotate([90,0,0])
						cylinder(d=15-1.75, h=2.75*2, center=true);
					}

					translate([0,2.75/2+7,0])
					difference(){
						translate([0,0,0])
						rotate([90,0,0])
						cylinder(d=15, h=2.75, center=true);
						translate([0,0,0])
						rotate([90,0,0])
						cylinder(d=15-1.75, h=2.75*2, center=true);
					}
				}*/
		
			}// end diff
		
			// mounting pad
			make_motor_mount_pan_pad(full=0, h=h);

		}

		// side cutoff
		make_motor_mount_pan_pad(full=1, h=h);

		// half join screw holes
		rotate([0,25,0]){
			color("blue")
			translate([-u*1.7,u*1.9,-u*1])
			rotate([-90,0,0])
			make_countersink();

			color("blue")
			translate([-u*1.7,u*1.9,u*1])
			rotate([-90,0,0])
			make_countersink();
		}

		// screw holes
		translate([0,-.25,1.5-.85])
		//translate([1.5,-.25,0])
		color("red"){
			translate([-u*1.75,u/2,u*3])
			rotate([0,90,0])
			make_countersink();
	
			translate([-u*1.75,-u/2,u*4])
			rotate([0,90,0])
			make_countersink();
	
			translate([-u*1.75,u/2,u*5])
			rotate([0,90,0])
			make_countersink();
		}

	}//end diff	

}

module make_motor_mount_pan_top(){
	s = 100;
	difference(){
		make_motor_mount_pan();

		color("blue")
		translate([
			0,
			-s/2-9.5+10-0.5,
			0])
		cube([s,s,s], center=true);
        
        // slot cutout for motor with encoder
        //translate([0,-u*3,0])
        rotate([0,25,0])
        translate([u*1,0,0])
        cube([u*3,u*5,u*2.1], center=true);
            
	}
}

module make_motor_mount_pan_bottom(){
	s = 100;
	difference(){
		make_motor_mount_pan();

		color("blue")
		translate([
			0,
			+s/2-9.5+10-0.5,
			0])
		cube([s,s,s], center=true);

	}
}


make_motor_mount_pan_top();
make_motor_mount_pan_bottom();
//make_motor_mount_pan();
