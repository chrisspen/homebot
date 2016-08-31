use <../openscad-extra/countersink.scad>;
use <../openscad-extra/strut.scad>;

module make_lrf_prop_base(d=100, r=7.14+0.5){
    
    difference(){
        union(){
            translate([0,0,1])
            cylinder(r1=d/2, r2=r, h=19, center=false, $fn=100);
            cylinder(d=d, h=1, center=false, $fn=100);
        }
        
        // upper hole
        translate([0,0,10])
        cylinder(d=r, h=100, $fn=100);
    
        // lower hole
        cylinder(d=2.5, h=100, $fn=100);
        
        // wire guide
        translate([0,0,-15/2+5])
        cube([5,150,15], center=true);
        
        // block cutout
        translate([0,0,5/2-1/2])
        cube([10+1,10+1, 5+1], center=true);
    }
}

module make_lrf_prop_base_block(){
    difference(){
        translate([0,0,3/2])
        cube([10, 10, 3], center=true);

        translate([0,0,0])
        rotate([0,180,0])
        make_countersink(inner=10, d1=2.5+0.5);
    }
}

module make_lrf_prop_top(r=7.14+0.5){
    difference(){
        translate([0,0,0])
        cylinder(d=r+2, h=10, $fn=100);
    
        // upper hole
        translate([0,0,-100+8])
        cylinder(d=r, h=100, $fn=100);
    }
    
    translate([0,0,30]){
        difference(){
            hull(){
                scale([1,.5,1])
                sphere(r=(r+2)/2, $fn=100);
                translate([0,0,-20]) cylinder(d=r+2, h=1, $fn=100);
            }
            translate([0,-20/2,0]) cube([30,20,60], center=true);
        }// end diff
    }
}

module make_lrf_prop_top2(r=7.1+0.5){
    
    difference(){
     
        union(){
            
            // main mass
            translate([0,0,0])
            cylinder(d=r+2, h=10, $fn=100);
            
            hull(){
                translate([-10/2 + 0.5,0,10/2])
                cube([1,20,10], center=true);
            
                translate([-2,0,0])
                scale([.5,1,1])
                cylinder(d=r+2, h=10, $fn=100);
            }
            
        }
    
        // upper hole
        translate([0,0,-100/2])
        cylinder(d=r, h=100, $fn=100);
        
        // break
        translate([5,0,0])
        cube([5,1,100], center=true);
    }
    
}

module make_lrf_centerpoint_nub_holes(d1=2.5+0.5){
    color("green")
    for(i=[-1:2:1])
    translate([0, 20*i, 1.5])
    make_countersink(d1=d1, d2=5.5, inner=30);
}

module make_lrf_centerpoint_nub(d1=2.5+0.5, offset=0){
    difference(){
            
        color("red")
        //for(j=[0:1])
        hull()
        for(i=[0:1])
        //mirror([1*i, 0, 0])
        translate([0,0,2*i])
        scale([1,1,3])
        rotate([90,0,0])
        cylinder(d=5, h=10*5, center=true, $fn=100);

        // bottom cutoff
        translate([0,0,-5]) cube([10,200,10], center=true);
        
        // screw holes
        translate([0, 0, 4 + offset])
        make_lrf_centerpoint_nub_holes(d1=d1);
    
    }// end diff
}

module make_lrf_centerpoint_nub_top(){
    make_lrf_centerpoint_nub(d1=2.5, offset=10);
}

module make_lrf_centerpoint(){
    
    difference(){
            
        color("blue")
        translate([0,0,-5/2]){
            rotate([0,90,0])    
            make_cross_plate(
                w=5*10,
                h=5*18,
                hub_d=0,//u*4,
                t=5,
                cross_type=1
            );
            
            rotate([0,90,0])    
            make_cross_plate(
                w=5*20,
                h=5*10,
                hub_d=0,//u*4,
                t=5,
                cross_type=1
            );
        }

        // side mount holes
        for(i=[0:1]) mirror([1*i, 0, 0])
        translate([8.5*5, 0, 1])
        make_lrf_centerpoint_nub_holes(d1=2.5);

        // front/back mount holes
        for(i=[-1:2:1])
        rotate([0,0,90])
        translate([8.5*5*i + 5*i, 0, 1])
        make_lrf_centerpoint_nub_holes(d1=2.5);
        
    }
    
    difference(){
        color("blue")
        intersection(){
            translate([0,0,-2.5])
            cylinder(d=130, h=5, center=true, $fn=100);
            
            translate([0, -100, -5/2])
            cube([10*5, 20*5, 10], center=true);
        }
        
        color("blue")
        intersection(){
            translate([0,0,-2.5])
            cylinder(d=130-12, h=10, center=true, $fn=100);
            
            translate([0, -100, -5/2])
            cube([10*5-10, 20*5, 20], center=true);
        }
    
        // camera alignment axis
        color("green")
        translate([0, -62, -5])
        //cylinder(d=5, h=500, center=true);
        rotate([0,180,0])
        make_countersink(d1=2.5, d2=5, inner=50);
            
    }
}

//translate([0,0,30]) make_lrf_prop_top();

//translate([0,0,30]) make_lrf_prop_top2();

//make_lrf_prop_base();

//translate([40,40,0]) make_lrf_prop_base_block();

if(0){
    import("../printable/ballast_tray_20160414.stl");
    mirror([0,1,0]) import("../printable/ballast_tray_20160414.stl");
}

if(1)
for(i=[0:1]) mirror([1*i, 0, 0])
translate([8.5*5, 0, 0])
make_lrf_centerpoint_nub();

make_lrf_centerpoint();
