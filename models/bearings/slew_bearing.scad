/*
Inspired by http://www.thingiverse.com/thing:885141
*/
include <../settings.scad>;
use <slew_bearing_generator_201508.scad>;
use <../plates/countersink.scad>;
use <../electronics/QRD1114.scad>;
use <../gears/pan_gears.scad>;
use <../plates/neck.scad>;
use <ThrustBearing.scad>;


module make_test_bearing(
    show_outer_race = 1,
    show_inner_race_a = 1,
    show_inner_race_b = 1,
    show_balls = 1
){

    od = 50;
    height = head_slew_bearing_height;

    //difference(){

    Bearing(
        outer = od/2,
        inner = od/2 - u*2,
        hole = od/2 - u*4,
        ball_radius = head_slew_bearing_ball_diameter/2,
        gap = head_slew_bearing_gap,
        height = height,
        outer_flange=0,
        slew=0,
        pr=0.1,
        show_outer_race=show_outer_race,
        show_inner_race_a=show_inner_race_a,
        show_inner_race_b=show_inner_race_b,
        show_balls=show_balls
    );
    
    color("blue"){
        for(i=[0:2]){
            rotate([0,0,120*i])
            translate([od/2 - u*1,0,height*.30]){
                //cylinder(d=screw_thread_diameter, h=head_slew_bearing_height*2, center=true);
                make_countersink();
            }
            
            rotate([0,0,120*i])
            translate([od/2 - u*3,0,height*.30]){
                //cylinder(d=screw_thread_diameter, h=head_slew_bearing_height*2, center=true);
                make_countersink();
            }
        }
    }

    //}//end diff
}

module make_neck_mount_holes(offset_z=0){

        translate([-neck_strut_spacing,0,offset_z])
        rotate([90,0,90])
        make_neck_struct_bottom_holes(cs=1);

        translate([neck_strut_spacing,0,offset_z])
        rotate([-90,180,90])
        make_neck_struct_bottom_holes(cs=1);

}

module make_joint_holes(){

    n = 2;
    m = 2;
    spacing = u*5;
    z = 1.5-3;

    for(i=[0:n]){
        for(j=[0:m]){
            if((i==0 && j==0) || (i==0 && j==m) || (i==n && j==0) || (i==n && j==m)){
            }else{
                color("green")
                translate([i*spacing - n*spacing/2, j*spacing - m*spacing/2 - u*0, 0]){
                    //cylinder(d=screw_thread_diameter, h=u*4, center=true);
                    translate([0,0,z])
                    rotate([0,180,0])
                    make_countersink();
                }
            }
        }
    }
/*
    translate([u*2,u*7,z])
    rotate([0,180,0])
    make_countersink();

    translate([-u*2,u*7,z])
    rotate([0,180,0])
    make_countersink();

    translate([u*2,-u*7,z])
    rotate([0,180,0])
    make_countersink();

    translate([-u*2,-u*7,z])
    rotate([0,180,0])
    make_countersink();
    */


}

module make_head_bearing(
    show_outer_race = 1,
    show_inner_race_a = 1,
    show_inner_race_b = 1,
    show_balls = 1,
    show_holes=1,
    show_gears=1,
    gap=0.2,
    show_body_mount=1
){

    collar_offset = u*0.25;
    collar_height = u+collar_offset;
    block_offset = u;
    
    
    difference(){
        /*
        Bearing(
            outer = head_slew_bearing_outer_diameter/2,
            inner = head_slew_bearing_inner_diameter/2,
            ball_radius = head_slew_bearing_ball_diameter/2,
            gap = head_slew_bearing_gap,
            height = head_slew_bearing_height,
            hole = head_slew_bearing_hole_diameter/2,
            outer_flange=0,
            slew=head_slew_bearing_slew_gap,
            pr=head_slew_bearing_gap_factor,
            show_outer_race=show_outer_race,
            show_inner_race_a=show_inner_race_a,
            show_inner_race_b=show_inner_race_b,
            show_balls=show_balls
        );
*/

        union(){
            
            ThrustBearing(
                outer_radius=head_slew_bearing_outer_diameter/2-u,//outside race size
                inner_radius=head_slew_bearing_inner_diameter/2-4,//inside race size
                inner_top_height_extra=0,//collar_offset,
                ball_radius=head_slew_bearing_ball_diameter/2,//size of ball
                ball_border=0.8,//wall thickness around ball
                gap = gap,//separation between races
                height = head_slew_bearing_height,//total height of all races
                hole = head_slew_bearing_hole_diameter/2,
                bottom_flange_thickness=0.25,
                show_outer=show_outer_race,
                show_inner_top=show_inner_race_a,
                show_inner_bottom=show_inner_race_b,
                $fn=100
            );
            
            if(show_outer_race){
            translate([0,0,-block_offset])
            difference(){
                translate([0,0,0])
                cube([
                    (head_slew_bearing_outer_diameter/2-u)*2,
                    (head_slew_bearing_outer_diameter/2-u)*2,
                    u
                ], center=true);
                
                cylinder(
                    //d=(head_slew_bearing_inner_diameter/2)*2-3,
                    d=(head_slew_bearing_inner_diameter/2-1.5)*2,
                    h=u*4, center=true, $fn=100);

                for(i=[-1:2:1]) for(j=[-1:2:1])
                color("red")
                translate([(u*12-u*2.5)*i, (u*12-u*2.5)*j, 0])
                cube([u*3, u*3, u*2], center=true);
                
            }
        }
            
        }
        
        if(show_outer_race){
          //make_bearing_horizontal_mount_holes();
            
            // mounting holes
            translate([0,0,-block_offset])
            color("red")
            for(i=[0:14]) for(j=[-1:2:1]) for(k=[0:1])
            rotate([0,0,90*k])
            translate([
                (head_slew_bearing_outer_diameter/2-u)*j,
                head_slew_bearing_outer_diameter/2-u*1.5 - u*i - u*3,
                0])
            rotate([0,90,0])
            cylinder(d=screw_thread_diameter, h=u*2, center=true);
            
            // notch mounting holes
            color("red")
            for(i=[0:1]) for(j=[-1:2:1]) for(k=[0:3])
            rotate([0,0,90*k])
            translate([u*8*j, u*9.5 - u*i, -u])
            rotate([0,90,0])
            cylinder(d=screw_thread_diameter, h=u*2, center=true);
            
        }

        if(show_holes){
            
            if(show_inner_race_b){
                translate([0,0,3.25])color("purple")make_neck_mount_holes(offset_z=u*.5);
            }else if(show_inner_race_a){
                translate([0,0,3.25])color("purple")make_neck_mount_holes();
            }
            
            // holes for screwing inner parts a and b together
            make_joint_holes();
            
            rotate([0,0,45])
            make_qrd1114_housing_holes();
            rotate([0,0,-45])
            make_qrd1114_housing_holes();
                    
            // sliping cable cutout
            translate([0,0,0])
            cube([7.2+tolerance, 13+tolerance, u*2], center=true);
                    
        }


    }//end diff

    // male alignment rings
    if(show_inner_race_a){
        color("green")
        for(i=[-1:2:1]) for(j=[0:1])
        rotate([0,0,90*j])
        translate([u*5*i,0,-1/2])
        difference(){
            cylinder(d=u-tolerance, h=1, center=true);
            cylinder(d=screw_thread_diameter+tolerance, h=u, center=true);
        }
    }
    
    // female alignment rings
    if(show_inner_race_b){
        color("red")
        for(i=[-1:2:1]) for(j=[0:1])
        rotate([0,0,90*j])
        translate([u*5*i,0,1/2])
        difference(){
            cylinder(d=u*2, h=1, center=true);
            cylinder(d=u+tolerance, h=u, center=true);
        }
    }
    
    if(show_outer_race && show_gears){
        difference(){
            translate([0,0,5])
            color("purple")
            make_pan_gears2(show_a=1, show_b=0, helpers=0);//big gear    
                
            // alignment marker cutout
            color("green")
            rotate([0,0,45])
            translate([u*9.5,0,u])
            cube([2,2,6], center=true);
        }
    }
    

    if(show_inner_race_b){

    /*            
                // outer shell housing for IR sensors
                for(i=[0:1])
                rotate([0,0,90*i])
                translate([-head_slew_bearing_inner_diameter/2+2+tolerance/2+1, 0, u])

                rotate([-90,180,90])
                color("blue")
                make_QRD1114_housing();
      */          
                
            //}

    }

    if(show_outer_race && show_body_mount){
    translate([0,0,-(collar_height+head_slew_bearing_height)/2]){
        color("blue")
        difference(){
            cylinder(
                d=head_slew_bearing_outer_diameter,
                h=collar_height,
                center=true,
                $fn=100);
            
            cube([70, 80*2, collar_height*2], center=true);//forward cutout
            cube([80*2, 55, collar_height*2], center=true);//side cutout
            
            translate([0,0,-2.25])
            cube([80, 80, collar_height], center=true);//center cutout
         
            cylinder(d=head_slew_bearing_inner_diameter+1, h=head_slew_bearing_height*2, center=true);   
            
            translate([-40-u,u*6,-head_slew_bearing_height/2-collar_offset-u/2+u*1.125])
            rotate([0,-90,0])
            make_countersink(bt_extra=1.95);
            
            translate([-40-u,-u*6,-head_slew_bearing_height/2-collar_offset-u/2+u*1.125])
            rotate([0,-90,0])
            make_countersink(bt_extra=1.95);
            
            translate([40+u,u*6,-head_slew_bearing_height/2-collar_offset-u/2+u*1.125])
            rotate([0,90,0])
            make_countersink(bt_extra=1.95);
            
            translate([40+u,-u*6,-head_slew_bearing_height/2-collar_offset-u/2+u*1.125])
            rotate([0,90,0])
            make_countersink(bt_extra=1.95);
        }   
    }
    }
    
}

module make_bearing_horizontal_mount_holes(d_offset=0, w_offset=0, as_countersinks=0, n=16, itotal=16, iadd=0, inner=100){
    for(i=[0:itotal-1]){
        color("red")
        rotate([0,0,360/n*(i+iadd)])
        translate([-57.5+d_offset,0,0]){
            rotate([0,-90,0])
            //cylinder(d=screw_thread_diameter, h=20, center=true);
            translate([0,0,10])
            make_countersink(inner=inner);
            if(as_countersinks==0)
            cube([u*2+w_offset,u+w_offset,u*2+w_offset], center=true);
        }
        
    }
}

/*
make_head_bearing(
    show_outer_race = 0,
    show_inner_race_a = 1,
    show_inner_race_b = 1,
    show_balls = 0,
    show_holes=1,
    show_gears=1,
    show_body_mount=0
);
*/

module make_qrd1114_housing_holes(){
    offset = u*1.25;
    translate([0,-u*8,u*1.38]){
        translate([-offset,0,0])
        make_countersink();
        translate([+offset,0,0])
        make_countersink();
    }
}

module make_qrd1114_housing_simple(show_sensor=0, light_width=1, type=1, qrd1114_offset_y=1){

    qrd1114_housing_height = 10;
    qrd1114_offset_z = -0.7/2;
    
    middle_light_ratio = 0.4;

    difference(){
        intersection(){
            translate([0,-44.75+1,4.75+.7]){
            
                if(show_sensor){
                    translate([0,0,qrd1114_offset_z])
                    rotate([90,0,0])
                    make_QRD1114();
                }
        
                difference(){
            
                    // main body
                    color("green")
                    translate([0,0,0])
                    cube([20,u*3,u*1],center=true);
        
                    // sensor cutout
                    translate([0 ,qrd1114_offset_y, qrd1114_offset_z])
                    rotate([90,0,0]){
                    
                        //make_QRD1114();
                        make_QRD1114_housing_bb(shell_thickness=0);
                    }
                    
                }// end diff
            }
        
            union(){
                // cylindrical face of the bulkheaad
                color("green")
                cylinder(d=95-tolerance, h=qrd1114_housing_height+5, $fn=100);
                
                // overhang lip to shade sensor opening
                color("blue")
                translate([0,0,7.96-.5])
                cylinder(d=95-tolerance+2*2, h=1, $fn=100);
            }
        }//end intersection
    
        make_qrd1114_housing_holes();
    }//end diff
    
    /*
    // top lip
    translate([0,-46.75,7.3])
    cube([5.5,.8,.5], center=true);
    */
    
    translate([0, -46.75+qrd1114_offset_y, 7.95 - qrd1114_housing_height*.25]){
        difference(){
            cube([5.5,.8,qrd1114_housing_height/2], center=true);
            color("red"){
                if(type==1){
                    //dot
                    translate([0,0,-.4/2])
                    cube([light_width,1,(qrd1114_housing_height/2-.4)*middle_light_ratio], center=true);
                }else{
                    //split
                    translate([0,0,-.4/2]){
                        difference(){
                            cube([light_width,1,(qrd1114_housing_height/2-.4+.1)/1], center=true);
                            cube([light_width+.2,1,(qrd1114_housing_height/2-.4)*middle_light_ratio], center=true);
                        }//end diff
                    }
                }
            }
        }//end diff
    }
    
}


//rotate([0,180,0]) make_qrd1114_housing_simple(type=1);
//translate([0,0,7]) rotate([0,180,0]) make_qrd1114_housing_simple(type=2);


//make_QRD1114_housing_bb(shell_thickness=0);

/*
make_head_bearing(
    show_outer_race = 1,
    show_inner_race_a = 0,
    show_inner_race_b = 0,
    show_balls = 0,
    show_holes=0,
    show_gears=1,
    show_body_mount=0
);
*/


make_head_bearing(
    show_outer_race = 0,
    show_inner_race_a = 0,
    show_inner_race_b = 1,
    show_balls = 0,
    show_holes=1,
    show_gears=1,
    gap=0.5,
    show_body_mount=0
);


/*
make_head_bearing(
    show_outer_race = 0,
    show_inner_race_a = 1,
    show_inner_race_b = 0,
    show_balls = 0,
    show_holes=1,
    show_gears=1,
    gap=0.5,
    show_body_mount=0
);
*/



//        make_bearing_horizontal_mount_holes();

/*
ThrustBearing(
   outer_radius = 10,//outside race size
   inner_radius = 5,//inside race size
   ball_radius=1,//size of ball
    ball_border=0.25,//wall thickness around ball
   gap = 0.1,//separation between races
   height = 4,//total height of all races
    bottom_flange_thickness=0.25,
    show_outer=1,
    show_inner_top=0,
    show_inner_bottom=0,
    $fn=100
);
*/


/*
color("blue")
translate([-head_slew_bearing_outer_diameter/2,0,0])
cube([head_slew_bearing_height,head_slew_bearing_height,head_slew_bearing_height], center=true);

color("blue")
translate([-head_slew_bearing_outer_diameter/2+u*3.5,0,0])
cylinder(d=screw_thread_diameter, h=50, center=true);

color("red")
translate([-head_slew_bearing_outer_diameter/2+u*0.5,0,0])
cylinder(d=screw_thread_diameter, h=50, center=true);
*/

/*
color("green")
import("../printable/bearings_slew_bearing_inner_bottom_20150927.stl");
*/

/*
make_test_bearing(
    show_outer_race = 1,
    show_inner_race_a = 1,
    show_inner_race_b = 1,
    show_balls = 0
);
*/