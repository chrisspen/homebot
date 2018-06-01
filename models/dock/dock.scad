
include <../settings.scad>;
use <../plates/recharge_mount.scad>;
use <../openscad-extra/src/rounded.scad>;
use <../openscad-extra/src/torus.scad>;
use <../openscad-extra/src/countersink.scad>;
use <../openscad-extra/src/wedge.scad>;

//18 gap between ground and bottom due to wheels

post_diameter = 7.14375; // 9/32"

module make_dock_join_hole(){
    rotate([0,45,0])
    cube([10+0.4,5+0.4,10+0.4], center=true);
}

module make_dock_join_sliver(){
    rotate([0,45,0])
    cube([10,5,10], center=true);
}

module make_bottom_dock_rim(holes=1, part=0){
    
    difference(){
        intersection(){
                
            difference(){
                union(){
                    difference(){
                        color("red")
                        cylinder(d=180, h=60, $fn=100);
                        
                        // center cutout
                        cylinder(d=160, h=200, center=true, $fn=100);
                        
                        // half cutout
                        translate([0,-200/2,0])
                        cube([200,200,200], center=true);
                            
                        // recharge cavity
                        translate([0,80,53])
                        //rounded_cube([55,50,40], r=5, center=true);
                        cube([55,50,40], center=true);
                        
                    }//end diff
                    difference(){
                        for(i=[0:1]){
                            mirror([i,0,0]){
                                
                                // angled wall
                                color("blue")
                                translate([-75-10,0,20])
                                rotate([45,0,0])
                                cube([10,128-0.75,128-0.75], center=true);
                                
                                // bottom straight wall
                                color("orange")
                                translate([-75-10,0,10])
                                cube([10,180,20], center=true);
                            }
                        }
                        
                        //topo cutoff
                        translate([0,0,100/2+60])
                        cube([200,200,100], center=true);
                        
                        //bottom cutoff
                        translate([0,0,-100/2])
                        cube([200,200,100], center=true);
                    }//end diff
                }

                // rubber band cutout
                translate([0,0,48])
                torus(r1=5, r2=80, $fn=150);

                // rubber band mount holes
                translate([0,0,48])
                rotate([0,90,0])
                cylinder(d=10, h=200, center=true, $fn=100);

                // rubber band mount holes
                translate([0,-45,48])
                rotate([0,90,0])
                cylinder(d=10, h=200, center=true, $fn=100);

                // rubber band mount holes
                for(i=[0:1])
                mirror([i,0,0])
                translate([90,-22.5,48])
                rotate([0,0,90])
                rotate([0,90,0])
                cylinder(d=10, h=46, center=true, $fn=100);

                // charging light hole
                translate([0,0,48])
                rotate([0,0,-45])
                rotate([0,90,0])
                translate([0,0,-176])
                cylinder(d1=60, d2=10, h=200, center=true, $fn=100);
                    
                // rubber band screw holes
                for(i=[0:1])
                mirror([i,0,0]){
                    translate([85,0,0]){
                        translate([0,0,60])
                        make_countersink(inner=50);
                        translate([0,-45,60])
                        make_countersink(inner=50);
                    }
                }
            
                // post holes
                for(i=[0:1])
                mirror([i,0,0])
                rotate([0,0,23])
                translate([0,85,100/2+54])
                cylinder(d=7.4, h=100, center=true, $fn=100);
            
                // mount holes for mat
                if(holes)
                for(j=[0:1])
                mirror([j,0,0])
                for(i=[0:8])
                translate([85, i*20 - 80, -0])
                rotate([180,0,0])
                make_countersink(inner=15);

                // strain gauge attachment
                translate([0,90,16])
                difference(){
                    cylinder(d=10, h=5, center=true, $fn=100);
                    cylinder(d=6, h=6, center=true, $fn=100);
                }
    
                // mass reduction holes, bottom row
                for(i=[0:7])
                if(i!=4)
                translate([0,-70+i*20,10])
                rotate([0,90,0])
                cylinder(d=15, h=200, center=true);

                // mass reduction holes, middle row
                for(i=[0:6])
                if(i!=4)
                translate([0,-60+i*20,20])
                rotate([0,90,0])
                cylinder(d=10, h=200, center=true);
                
                // mass reduction holes, top row
                for(i=[0:5])
                if(i!=3)
                translate([0,-50+i*20,30])
                rotate([0,90,0])
                cylinder(d=15, h=200, center=true);
                

            }//end diff
            
            //part0
            if(part==0)
            cube([1000,1000,1000], center=true);
            
            //part1
            if(part==1)
            union(){
                translate([100/2,-100/2,0])
                cube([100,100,150], center=true);
                translate([0,0,0])
                rotate([0,0,-85])
                wedge(h=150, r=150, a=10);
            }
            
            //part2
            if(part==2)
            translate([0,0,0])
            rotate([0,0,-55])
            wedge(h=150, r=150, a=50);
            
            //part3
            if(part==3)
            translate([0,0,0])
            rotate([0,0,0])
            wedge(h=150, r=150, a=60);
            
            //part4
            if(part==4)
            translate([0,0,0])
            rotate([0,0,55])
            wedge(h=150, r=150, a=50);
            
            //part5
            if(part==5)
            union(){
                translate([-100/2,-100/2,0])
                cube([100,100,150], center=true);
                translate([0,0,0])
                rotate([0,0,85])
                wedge(h=150, r=150, a=10);
            }
            
        }// end intersection
        
        if(part==2){
            rotate([0,0,-30])
            translate([0,85,25])
            make_dock_join_hole();
            rotate([0,0,-30-50])
            translate([0,85,25])
            make_dock_join_hole();
        }
        
        if(part==4){
            rotate([0,0,30])
            translate([0,85,25])
            make_dock_join_hole();
            rotate([0,0,-(-30-50)])
            translate([0,85,25])
            make_dock_join_hole();
        }
        
    }//end diff
    
    if(part==1){
        rotate([0,0,-30-50])
        translate([0,85,25])
        make_dock_join_sliver();
    }
    
    if(part==3){
        rotate([0,0,-30])
        translate([0,85,25])
        make_dock_join_sliver();
        rotate([0,0,30])
        translate([0,85,25])
        make_dock_join_sliver();
    }
    
    if(part==5){
        rotate([0,0,-(-30-50)])
        translate([0,85,25])
        make_dock_join_sliver();
    }
    
    // mount holes for mat
    if(!holes)
    for(j=[0:1])
    mirror([j,0,0])
    for(i=[0:8])
    translate([85, i*20 - 80, -0])
    rotate([180,0,0])
    make_countersink(inner=15);

    // mock footprint
    if(0)
    cube([180,180,10], center=true);
}

module make_dock_retainer(){
    
    // center post
    translate([0,0,0])
    difference(){
        color("blue")
        translate([0,-7.5,0])
        cube([5,35,5], center=true);
        
        translate([0,-22.5,0])
        cylinder(d=3, h=50, center=true);
    }
    
    // side posts
    for(i=[0:1])
    mirror([i,0,0])
    translate([20,-5,0])
    difference(){
        color("blue")
        translate([0,-7.5,0])
        cube([5,35,5], center=true);
        
        translate([0,-22.5,0])
        cylinder(d=3, h=50, center=true);
    }
    
    // outer restraining arms
    intersection(){
        translate([0,-80,0])
        difference(){
            cylinder(d=180+10, h=5, center=true, $fn=150);
            cylinder(d=180, h=10, center=true, $fn=150);
        }
        cube([80,50,50], center=true);
    }
    
    // middle center flange
    difference(){
        color("orange")
        translate([0,0,-2.5])
        linear_extrude(5)
        polygon([[27,7],[0,11],[-27,7],[-22,-20],[22,-20]]);
        
        for(i=[0:1])
        mirror([i,0,0]){
            translate([10,-11,0])
            cube([5, 20, 10], center=true);
            
            //wire strap cutouts
            for(j=[-1:2:1])
            translate([10+3.5*j,5,0])
            cube([2,3.5,20], center=true);
        }
    }
    
    
}

module make_dock_mast_head(){
    
    color("purple")
    
    difference(){
        
        for(j=[0:1])
        translate([0,0,(-66.5+5)*j])
        for(i=[0:1])
        mirror([i,0,0])
        rotate([0,0,23])
        translate([0,85,0])
        difference(){
            cylinder(d=10, h=5, center=true);
            cylinder(d=7.2, h=10, center=true);
        }
        
        // ring stress cutouts
        for(i=[-1:2:1])
        translate([33*i,82,-30])
        cube([2,5,100], center=true);
    }

    color("green")
    translate([0,74-0.1,-66.5/2+5/2])
    cube([66.5, 1.3, 66.5], center=true);


}

module dock_mast_recessor(){
    
    offset_y = 75;
    
    difference(){
        
        // main mass
        color("orange")
        translate([0,111+4+2-2-10/2+1,55+5/2])
        cube([78, 85+10+5, 10], center=true);
     
        difference(){
            color("red")
            cylinder(d=180+0.5, h=60, $fn=100);
            
            // center cutout
            cylinder(d=120, h=200, center=true, $fn=100);
            
            // half cutout
            translate([0,-200/2,0])
            cube([200,200,200], center=true);

        }//end diff
    
        // recharge cavity
        translate([0,95+5,53])
        cube([55,100,40], center=true);
    
        // center cutout
        cylinder(d=160, h=200, center=true, $fn=100);    

        // post holes
        color("red")
        translate([0,80,0])
        for(i=[0:1])
        mirror([i,0,0])
        rotate([0,0,23])
        translate([0,85,100/2])
        cylinder(d=7.4+0.25/2, h=100, center=true, $fn=100);

        // side arch cutout
        translate([0,0,52.5])
        scale([1,1,.2])
        rotate([0,90,0])
        translate([0,120.5,0])
        cylinder(d=60, h=150, center=true, $fn=100);
        
        // back arch cutout
        translate([0,0,52.5])
        scale([1,1,.2])
        translate([0,120.5,0])
        rotate([0,0,90])
        rotate([0,90,0])
        cylinder(d=55, h=150, center=true, $fn=100);

    }// end diff
    
    // post nubs
    color("blue")
    translate([0,0,0])
    for(i=[0:1])
    mirror([i,0,0])
    rotate([0,0,23])
    translate([0,85,100/2+54-93/2+1])
    cylinder(d=7.4-0.5/2, h=100-93, center=true, $fn=100);
    

}

torso_height = 155;

// mock torso
translate([0,-150/2+75-75,18])
if(0){
    translate([0,0,torso_height/2])
    cylinder(d=150, h=torso_height, center=true);

    translate([0,0,230])
    sphere(d=150);

    color("red")
    translate([0,150/2-5,10+20])
    rotate([0,180,0])
    rotate([0,0,-45])    
    import("../printable/recharge_plug_female_v2_20160519.stl");
}

if(0)
color("gray")
translate([0,0,-5/2+1])
cube([300,300,5], center=true);

if(0)
translate([0,25,0])
translate([0,150/2-20-30,48])
make_recharge_plug_male(half=0);

if(0)
make_bottom_dock_rim(holes=0, part=0);

if(1){
    import("../printable/dock_rim_part1.stl");
    import("../printable/dock_rim_part2.stl");
    import("../printable/dock_rim_part3.stl");
    import("../printable/dock_rim_part4.stl");
    import("../printable/dock_rim_part5.stl");
}

if(1)
translate([0,0,0])
dock_mast_recessor();

if(0)
translate([0,0,100])
//make_dock_mast_head();
import("../printable/dock_masthead.stl");

// mock masts
if(0)
for(i=[0:1])
mirror([i,0,0])
rotate([0,0,23])
translate([0,85,0])
cylinder(d=7.3, h=150);

if(0)
translate([0,80+5-5,45+3])
make_dock_retainer();
