include <../settings.scad>;

module make_QRD1114(leads=1, shell_thickness=0){
	union(){
	    cube([qrd1114_w+shell_thickness, qrd1114_l+shell_thickness, qrd1114_h+shell_thickness], center=true);
	    if(leads){
    		for(i=[-1:2:1]){	for(j=[-1:2:1]){
    			translate([2.54/2*i, 2.11/2*j, -12.7/2-4.65/2])
    			cylinder(d=0.51, h=12.7, center=true);
    		}}
		}
	}
}

module make_QRD1114_housing_bb(shell_thickness=1){
    t = tolerance;
    union(){
        // main body
        cube([qrd1114_w+shell_thickness+t, qrd1114_l+shell_thickness+t, qrd1114_h+shell_thickness+t], center=true);
        
        // lead bb
        cube([
            qrd1114_w+t-1.5, // width
            qrd1114_l+shell_thickness+t, // height
            qrd1114_h+15+t // depth
        ], center=true);
    }
}

module make_QRD1114_housing(shell_thickness=1){
	t = tolerance;
    difference(){

        // main body
        make_QRD1114_housing_bb();

        // interior hollowing
        cube([qrd1114_w+t, qrd1114_l+t, qrd1114_h+t], center=true);
        
        // top cutout
        translate([0,shell_thickness*2,0])
        cube([qrd1114_w+t-0.5, qrd1114_l+t, qrd1114_h+t], center=true);
        
        // front/back cutout
        translate([0,0.5,0])
        cube([qrd1114_w+t-1.5, qrd1114_l+t+1, (qrd1114_h+t)*2], center=true);

    }
}

make_QRD1114();
//make_QRD1114_housing();
