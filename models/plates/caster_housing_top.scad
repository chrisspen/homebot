use <../openscad-extra/src/countersink.scad>;

module make_caster_screw_holes(){
    for(i=[-1:2:1])
    translate([15,10*i,2.6])
    make_countersink(d1=2.5, d2=5);
}

module make_caster_housing_top(){
    difference(){
        color("orange")
        cube([35,30,5], center=true);
        
        // cutout for round top
        color("blue")
        translate([-2.5,0,0])
        cylinder(d=22.5, h=30, center=true, $fn=100);
        
        // cutout for round bottom
        color("blue")
        translate([-2.5,0,-3.75])
        cylinder(d=26, h=10, center=true, $fn=100);
        
        // cutout for bottom slide
        translate([30,0,-.01])
        rotate([-90,0,90])
        color("red")
        linear_extrude(50)
        polygon([[0,0],[13,0],[13-1,2.5],[-13+1,2.5],[-13,0]]);
    
        make_caster_screw_holes();
    }
}

module make_caster_housing_bottom(){
    difference(){
        translate([35/2,0,-.01])
        rotate([-90,0,90])
        color("red")
        linear_extrude(35)
        polygon([[0,0],[13-0.25,0],[13-1-0.25,2.5],[-13+1+0.25,2.5],[-13+0.25,0]]);
        
        // cutout for thrust bearing 12mm diameter and 2mm height
        cube([40,12,1.5], center=true);
        
        // cutout for center thrust bearing
        color("purple")
        translate([-2.5,0,0])
        cylinder(d=12, h=2, center=true, $fn=100);
        
        make_caster_screw_holes();
    }
}

module make_caster_housing_center(){
    difference(){
        union(){
            // cutout for round top
            color("purple")
            translate([-2.5,0,1.6])
            cylinder(d=22.5-1, h=2, center=true, $fn=100);
            
            // cutout for round bottom
            color("purple")
            translate([-2.5,0,0.25])
            cylinder(d=26-1, h=0.75, center=true, $fn=100);
        }
            
        // cutout for center thrust bearing
        color("purple")
        translate([-2.5,0,0])
        cylinder(d=12, h=1.9, center=true, $fn=100);
    }
    difference(){
        intersection(){
            translate([2,0,0])
            scale([1.25,1,1.1])
            sphere(d=25, $fn=100);
                
            color("purple")
            translate([-2.5,0,52])
            cylinder(d=22.5-1, h=100, center=true, $fn=100);
        }
        
        // wheel well
        translate([6,0,12.5])
        cube([20,7,20], center=true);
        
        // axle hole
        color("red")
        translate([4,0,10])
        rotate([90,0,0])
        cylinder(d=2, h=50, center=true, $fn=100);
    }
    
}

module make_caster_wheel(d=11){
    rotate([90,0,0]){
        difference(){
            cylinder(d=d, h=5, center=true, $fn=100);
            cylinder(d=2.5, h=20, center=true, $fn=100);
        }
        //color("blue")
        //cylinder(d=1, h=100, center=true, $fn=100);
    }
}

rotate([180,0,0])
make_caster_housing_top();
