/*
2015.9.5 CKS
*/

module torus(r1=1, r2=2, $fn=50){
    rotate_extrude(convexity=10, $fn=$fn)
    translate([r2, 0, 0])
    circle(r=r1, $fn=$fn);
}

module ThrustBearing(
    outer_radius = 10,//outside race size
    inner_radius = 5,//inside race size
	inner_top_height_extra=0,
    ball_radius=1,//size of ball
    ball_border=0.25,//wall thickness around ball
	hole=1,//radius of center hole
    gap = 0.1,//separation between races
    height = 4,//total height of all races
    bottom_flange_thickness=0.25,
    show_outer=1,
    show_inner_top=1,
    show_inner_bottom=1,
    $fn=50
){

    tapered_gap = gap*1.4142;

    difference(){
        union(){
    
            // outer race
            if(show_outer){
                color("blue")
                difference(){
                    cylinder(d=outer_radius*2, h=height, center=true, $fn=$fn);//main
                    cylinder(d=inner_radius*2+gap, h=height+1, center=true, $fn=$fn);//inner cutout
    
                    translate([0,0,height/2-gap/2])
                    cylinder(d=inner_radius*2+(ball_radius+ball_border)*4+gap, h=height, center=true, $fn=$fn);//rim cutout
    
                    // tapered flange cutout
                    translate([0,0,-height/8-height/4])
                    cylinder(d2=inner_radius*2+tapered_gap, d1=inner_radius*2+ball_radius*2+ball_border*2+tapered_gap, h=height/4, center=true);
    
                }
            }

            // inner race top
            if(show_inner_top){
                color("red")
                union(){
                    translate([0,0,height/4])
                    cylinder(d=inner_radius*2-gap, h=height/2, center=true);//center
                    translate([0,0,height/4+gap/2+inner_top_height_extra/2])
                    cylinder(d=inner_radius*2+(ball_radius+ball_border)*4-gap, h=height/2-gap/2+inner_top_height_extra, center=true);//top flange
                }
            }

            // inner race bottom
            if(show_inner_bottom){
                color("green")
                translate([0,0,0])
                union(){
                    translate([0,0,-height/4])
                    cylinder(d=inner_radius*2-gap, h=height/2, center=true);//center
                    translate([0,0,-height/8-height/4])
                    cylinder(d2=inner_radius*2-tapered_gap, d1=inner_radius*2+ball_radius*2+ball_border*2-tapered_gap, h=height/4, center=true);//tapered flange
                }
            }
    
        }

        // ball cutout
        torus(r1=ball_radius, r2=inner_radius+ball_radius+ball_border, $fn=$fn);

		// center hole cutout
		if(hole)
		cylinder(d=hole*2, h=height*2, center=true);

        //translate([-outer_radius,0,0])
        //cube([outer_radius*2,outer_radius*2,outer_radius*2],center=true); 
    }

}

ThrustBearing(
	outer_radius = 10,//outside race size
    inner_radius = 5,//inside race size
	inner_top_height_extra=1,
    ball_radius=1,//size of ball
    ball_border=0.25,//wall thickness around ball
    gap = 0.1,//separation between races
    height = 4,//total height of all races
    bottom_flange_thickness=0.25,
    show_outer=1,
    show_inner_top=1,
    show_inner_bottom=1,
    $fn=50
);

