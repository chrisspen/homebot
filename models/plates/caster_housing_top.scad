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

module make_caster_housing_center(axle_d=2){
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
        if(1){
            color("blue")
            translate([6-4,0,20])
            cube([20,7,20], center=true);
            
            
            translate([4,0,10])
            rotate([90,0,0])
            cylinder(d=11+2, h=7, center=true, $fn=50);
        }
        
        // axle hole
        color("red")
        translate([4,0,10])
        rotate([90,0,0])
        cylinder(d=axle_d, h=50, center=true, $fn=100);
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

module make_caster_set_screw(){
    translate([0,0,6.75/2-1.8/2])
    cylinder(d=3.8, h=1.8, center=true, $fn=100);
    cylinder(d=2, h=6.75, center=true, $fn=100);
}

module make_caster_spring_cutouts(){
    // spring cutouts
    for(i=[-1:2:1])
    translate([2,-6.4*i,5-0.5])
    difference(){
        cylinder(d=5.85, h=7, center=true, $fn=50);
        cylinder(d=3.8, h=20, center=true, $fn=50);
    }
}

module make_caster_rounded_divider(d_offset=0){
    translate([-2.5,0,4])
    rotate([90,0,0])
    cylinder(d=16.5-d_offset, h=100, center=true, $fn=100);
}

module make_caster_axle_hole(d=0.5){
    translate([-2.5,0,3.6+0.3])
    rotate([90,0,0])
    cylinder(d=d, h=50, center=true, $fn=100);
}

module make_caster_housing_center_top(show_slot=0){
    difference(){
        intersection(){
            intersection(){
                intersection(){
                    make_caster_housing_center(axle_d=2.5);
                    translate([0,0,-(-50/2-2.6)])
                    cube([50,50,50], center=true);
                }
                
                translate([-0.8,0,0])
                rotate([0,-32,0])
                translate([50/2-10,0,-(-50/2-2.6-.5)])
                cube([50,50,50], center=true);
            }//end inter
    
            // forward guard
            union(){
                translate([5,0,10])
                cube([20,30,30], center=true);
                color("red")
                make_caster_rounded_divider(d_offset=0.5);
            }
        }

        
        // axle hub
        translate([-5/2,0, 3.6-0.1+0.3])
        rotate([90,0,0])
        cylinder(d=2.5+0.5, h=18+0.5, center=true, $fn=100);

        // axle slot
        if(show_slot)
        color("red")
        hull()
        for(i=[0:1])
        translate([-21+43/2,6+2-1,10+i*3-0.2])
        rotate([0,0,90])
        rotate([90,0,0]){
            translate([0,0,7/2])
            sphere(d=2.1, $fn=50);
            cylinder(d=2.1, h=7, center=true, $fn=100);
        }
    
        make_caster_spring_cutouts();
        
        color("green")
        translate([0,0,-0.1])
        make_caster_axle_hole();
        
    }//diff


    //make_caster_set_screw();

}

module make_caster_housing_center_bottom(){
    difference(){
        intersection(){
            make_caster_housing_center();
    
            union(){
                // bottom mass
                translate([0,0,-50/2+2.6])
                cube([50,50,50], center=true);
        
                // forward guard
                difference(){
                    translate([-15,0,-5.5-1])
                    cube([20,30,30], center=true);
                    color("red")
                    make_caster_rounded_divider(d_offset=-0.5);
                }
            }
            
        }//end inter
    
        make_caster_spring_cutouts();

    }// end diff
    
    difference(){
    
        // axle hub
        if(1)
        translate([-5/2,0,3.6+0.3])
        rotate([90,0,0])
        cylinder(d=2.5, h=18, center=true, $fn=100);
        
        color("green")
        make_caster_axle_hole(d=1);
    }
    
    // axle pedestal
    translate([-2.5,0,2.75])
    cube([1,18,0.5], center=true);

}

translate([2.5,0,0]){
    if(0)
    rotate([180,0,0])
    make_caster_housing_top();

    if(0)
    translate([0,0,0])
    rotate([180,0,0])
    make_caster_housing_bottom();

    if(0)
    translate([0,-30,0])
    make_caster_housing_center();

    if(0)
    translate([0,0,0+0.1+0])
    make_caster_housing_center_top();

    if(1)
    make_caster_housing_center_bottom();

    translate([4,0,10])
    rotate([90,0,0])
    import("../printable/caster_housing_wheel_d11.stl");
}
