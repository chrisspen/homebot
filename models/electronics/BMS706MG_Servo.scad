
module make_BMS706MG_servo_holes(d=4.5){
        //mount holes
        for(i=[-1:2:1]){
            for(j=[-1:2:1]){
                color("blue")
                translate([50/2*i,10/2*j,0])
                cylinder(d=d, h=30, center=true);
            }
        }
}

module make_BMS706MG_servo_horn_round_holes(hole_space=1.25, hole_d=2, hole_h=10, a=1, b=1, c=1){

            for(i=[0:3])
            for(j=[0:2])
            color("blue")
            rotate([0,0,90*i])
            translate([16-(hole_space+2)*j,0,0])
            if((j==0 && a==1) || (j==1 && b==1) || (j==2 && c==1))
            cylinder(d=hole_d, h=hole_h, center=true);
}

module make_BMS706MG_servo_horn_round(bb=0, d_tolerance=0, d_extra=0){

	hole_space = 1.25;

    if(bb){

        cylinder(d=45+d_extra, h=2.56*2);

    }else{
    	difference(){
    		union(){
    		
    			cylinder(d=24.65, h=2.56);
    		
    			translate([0,0,0])
    			cylinder(d=8.5+d_tolerance, h=6.75);
    		
    			for(i=[0:3])
    			hull(){
    				rotate([0,0,90*i])
    				translate([16,0,0])
    				cylinder(d=5.8, h=2.56);
    				cylinder(d=13, h=2.56);
    			}
    	
    		}
    	   /*
    		for(i=[0:3])
    		for(j=[0:2])
    		color("blue")
    		rotate([0,0,90*i])
    		translate([16-(hole_space+2)*j,0,-1])
    		cylinder(d=2, h=5);
    		*/
    		make_BMS706MG_servo_horn_round_holes(hole_space=hole_space);
    	
    		color("blue")
    		translate([0,0,-10+1.55])
    		cylinder(d=5.9, h=10);
    	
    		cylinder(d=3, h=100, center=true);
    	}
	}
}

module make_BMS706MG_servo(horn=0, d_tolerance=0){

	nub_offset_x = 43/2-9.3;

	translate([-nub_offset_x,0,0])
	difference(){
		union(){
			//main body
			//color("green")
			cube([43.5,21.5,16], center=true);
		
			//mount tabs
			translate([0,0,16/2+2.5/2])
			cube([54.5,21.5,2.5], center=true);
		
			//top body
			//color("red")
			translate([0,0,5/2+16/2+2.5])
			cube([43,21.5,5], center=true);
		
			//nub
			color("red")
			translate([nub_offset_x,0,16/2+2.5+5])
			cylinder(d=6+d_tolerance, h=3.75);
		}
		make_BMS706MG_servo_holes();
	}

	if(horn){
		
		translate([0,0,30-7.75])
		rotate([180,0,0])
		make_BMS706MG_servo_horn_round(bb=0, d_tolerance=d_tolerance);

	}

}


make_BMS706MG_servo(horn=1);
