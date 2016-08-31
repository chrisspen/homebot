include <../settings.scad>;
use <plate.scad>;
use <../plates/countersink.scad>;
use <cross_plate.scad>;
use <../electronics/BMS706MG_Servo.scad>;
use <../nuts.scad>;

module make_head_strut(holes=1, cross_type=2, pivot_d=0, hub_d=u*8, fv=0, fh=0, ev=0, eh=0){
    u = 5;
    t = u;
    w = u*20;
    h = u*15;

    n_holes_h_face = 6; // holes along the height of the face
    n_holes_w_face = 9; // holes along the width of the face

/*
    //sohcahtoa
    angle = atan((h-u)/(w-u));

    //c^2 = a^2 + b^2
    diagonal = sqrt(pow(w-u, 2) + pow(h-u, 2));

    // border
    difference(){
        cube([u,w,h], center=true);
        cube([u*2,w-u*2,h-u*2], center=true);
    }

    // cross beam
    for(i=[0:1]){
        mirror([0,0,i])
        translate([0, -w/2+u/2, -h/2+u/2])
        rotate([angle, 0, 0])
        translate([-u/2, 0, -u/2])
        cube([u, diagonal, u], center=false);
    }
    
    rotate([0,90,0])
    cylinder(d=u*4, h=u, center=true);
*/
    difference(){
        make_cross_plate(
            w=w,
            h=h,
//            hub_d=u*4,
            hub_d=hub_d,//u*8,
            t=t,
            cross_type=cross_type
        );
    
        for(i=[-1:2:1]){
            color("red")
            translate([0, i*w/2, 0])
            rotate([45, 0, 0])
            rotate([0, 90, 0])
            cube([2,2,t*2], center=true);
        }

        if(pivot_d)
        rotate([0,90,0])
        cylinder(d=pivot_d, h=50, center=true);
    
        if(holes){

            // side holes
            color("red"){
            translate([u,0,0]){
                // vertical
                for(j=[-1:2:1]){
                    for(i=[0:n_holes_h_face-1]){
                        translate([fv, (-w/2+u/2)*j, u*2*i - u*(n_holes_h_face-1)])
                        rotate([0,90,0])
                        make_countersink(outer=u, inner=u*1.5);
                    }
                }
                // horizontal
                for(j=[-1:2:1]){
                    for(i=[0:n_holes_w_face-1]){
                        translate([fh, u*2*i - u*(n_holes_w_face-1), (h/2-u/2)*j])
                        rotate([0,90,0])
                        make_countersink(outer=u, inner=u*1.5);
                    }
                }
            }
        
            // edge holes, front/back
                for(j=[0:1]){
                    for(i=[0:n_holes_h_face]){
                        translate([0, (-w/2+u/2)-u+1.5 + (w+u-3)*j + ev*j, u*2*i - u*(n_holes_h_face)])
                        rotate([0,90,-90+180*j])
                        make_countersink(outer=u, inner=u*1.1);
                    }
                }
        
            // edge holes, top/bottom
                for(j=[-1:2:1]){
                    for(i=[0:n_holes_w_face]){
                        translate([0, u*2*i - u*(n_holes_w_face), (h/2+u*.25)*j + eh*j])
                        rotate([0, (j==1)?0:180, 0])
                        make_countersink(outer=u, inner=u*1.5);
                    }
                }
        
            }//end red

        }

    }//end diff

}

module make_head_strut_servo_side(){
    difference(){
        make_head_strut(
            holes=1,
            cross_type=2,
            pivot_d=9,
            hub_d=u*8,
            fv=-3.3,
            fh=-3.3
        );
    
        // servo horn countersink
        color("blue")
        translate([u/2,0,0])
        rotate([0,90,0])
        cylinder(d=39, h=u, center=true, $fn=100);

        //screw hole
        translate([0,0,0])
        rotate([0,90,0])
        make_BMS706MG_servo_horn_round_holes(a=0, b=1, c=0, hole_h=u*2);
            
    }

}

module make_head_strut_open_side(holes=1){
    
    y_offset_center = 22.5;
    
    difference(){
        make_head_strut(
            holes=holes,
            cross_type=2,
            //pivot_d=screw_thread_diameter,
            hub_d=u*7,
            fv=-3.3,
            fh=-3.3
        );
            
        rotate([90,0,0])
        rotate([0,90,0]){
            
            // pivot hole
            color("red")
            translate([0,0,-u*.3])
            rotate([0,180,0])
            cylinder(d=u*5, h=u*3, center=true, $fn=100);
        
            /*/ centering notch
            translate([0, 0 + u*2.45, 0])
            rotate([0,0,45])
            cube([2+0.5,2+0.5,20], center=true);
            */
        }

        // wire strain gauge mount hole
        translate([0,0,5+2.5])
        rotate([90,0,0])
        rotate([0,90,0])
        color("blue"){
    
            translate([0,5+2.5,0])
            rotate([0,90,0])
            cylinder(d=2.5, h=15, center=true);
            
            for(i=[-1:2:1])
            translate([7.5*i,5+2.5,0])
            scale([2,1,1])
            cylinder(d=2.5, h=50, center=true);
        }
        
        /*/nut countersink
        color("red")
        translate([-(u/2-3.5/2+0.01),0,0])
        rotate([0,90,0])
        make_nut_2_5(h=3.5);
        */

    }// end diff

}

make_head_strut_open_side(holes=0);
