use <../electronics/mic.scad>;
use <../openscad-extra/src/wedge.scad>;
use <../openscad-extra/src/countersink.scad>;

module mic_layout(){
    angle = 25;
    translate([0,0,1.2])
    for(i=[0:1]){
        color("blue")
        rotate([0, 0, angle-angle*2*i])
        //translate([0, 0, 7.25*i + 5.25*(1-i)])
        translate([0, 0, 5.25])
        translate([0, -45+17-4, 0])
        //rotate([0,180*i,0])
        rotate([0,0,180])
        make_mic(tol=0.5, condenser_length=20);
        
        /*
        color("green")
        rotate([0,0,angle-angle*2*i])
        translate([0,-55,6.25])
        rotate([90,0,0])
        scale([2,1,1])
        cylinder(r1=6/2, r2=7.5/2, h=10, center=true, $fn=100);
        */
    }
}

module make_mic_mount(bb=0, tol=0){

    height = 10 + tol - 1;
    
    difference(){
        intersection(){
            difference(){
                
                // horizontal bb
                translate([0, -40, 5])    
                translate([0,45,5/2])
                rotate([0,0,180])
                wedge(h=height+1, r=70, a=64);
                
            }
            
            translate([0, -50, 10-2.5])
            cube([80, 30, height], center=true);
           
            translate([0,0,-35])
            sphere(d=150-2*2-1+tol, $fn=100);
        }
        
        if(!bb)
        mic_layout();

        if(!bb)
        color("red")
        for(i=[-1:2:1])
        translate([(15)*i, -47.5, 12.5-.5])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=14);
    }
    
        
    
}

//*
difference(){
    translate([0,30,-2.5])
    make_mic_mount();
    translate([0,0,100/2 + 5])
    cube([100,100,100], center=true);
}

rotate([0,180,0])
translate([0,0,-10])
difference(){
    translate([0,60,-2.5])
    make_mic_mount();
    translate([0,0,-(100/2 - 5)])
    cube([100,100,100], center=true);
}
//*/

//translate([0,30,0]) make_mic_mount(bb=1);

//*

//intersection(){

/*
color("gray")
translate([-0,0,-35])
import("../printable/head_shell_top.stl");
*/

//make_mic_mount(tol=0., bb=0);

//mic_layout();
//}

//color("gray") import("../printable/head_top.stl");

/*
color("gray")
translate([0,-80,-35])
import("../printable/head_shell_front.stl");
*/
