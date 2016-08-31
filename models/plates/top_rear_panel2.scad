include <../settings.scad>;
use <countersink.scad>;
use <../rounded_cube.scad>;

module _make_front_slice_bb(width=u*30, height=u*7, thickness=0, h_extend=0, back_extend=0){
    intersection(){
        color("orange")
        cylinder(d=150, h=height+h_extend, center=true, $fn=100);
        
        color("blue")
        translate([0, u*12-back_extend, 0])
        cube([width, u*8 + back_extend*2, height*2+h_extend], center=true);
    }
}

module _make_front_slice_shell(height=1, thickness=2){
    difference(){
        color("purple")
        translate([0, -u*13.5 + y_offset, 0])
        _make_front_slice_bb(height=height);
        
        // interior rim cutout
        translate([0,0,0])
        cylinder(d=150-thickness*2, h=height+1, center=true, $fn=75);
            
    }
    
}

module make_top_rear_panel2(height=u*11, thickness=2, show_door=1){
        
    difference(){
        union(){
            _make_front_slice_shell(height=height, thickness=thickness);
            
            // mount struts
            intersection(){
                _make_front_slice_bb(height=height, thickness=thickness);
                for(i=[-1:2:1]) for(j=[-1:2:1]){
                    translate([u*8.5*i, u*11, (-u - height/2 + u*2)*j]){
                        //cube([u, u*6, u*2], center=true);
                        
                        translate([0, -u*3, 0])
                        linear_extrude(height=u*10, center=true)
                        polygon(points=[[u/2, 0], [u/2, u], [u*1.25, u*5], [-u*1.25, u*5], [-u/2, u], [-u/2, 0]]);
                    }
                }
            }// end intersection
            
            // power button mount bulk
            translate([0,72,25-u*5])
            rotate([90,0,0])
            cylinder(r1=10/2, r2=3/2, h=4, center=true);
            
        }
    
        // mount holes
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([u*8.5*i, u*12+1-5, (height/2 - u)*j])
        rotate([-90, 0, 0])
        make_countersink(
            inner=20,
            outer=u*3,
            d1=screw_thread_diameter+0.5,
            d2=screw_head_diameter+0.5
        );
        
        // arch cutout
        color("red")
        translate([0,40,0])
        scale([1,6,7])
        rotate([0,90,0])
        cylinder(d=u*1, h=u*19, center=true);
        
        // bottom flat cutout
        color("green")
        translate([0,40+u/2,-u*3-u/2+u*3/2])
        cube([u*19, u*5, u*4], center=true);
        
        // power button cutout
        translate([0,75,25-u/2])
        rotate([90,0,0])
        cylinder(d=2, h=u*2, center=true);
        
        // power button mount screw hole
        translate([0, 72-2, 25-u*5])
        rotate([90,0,0])
        cylinder(d=2, h=5, center=true);
        
        // button slot cutout
        translate([0, 72.5-0.5/2, 4])
        cube([3+0.5, 1+0.5, u*2], center=true);
    
    }//end diff
}

module make_back_button_shim(length=13){
    color("green")
    translate([0, 72.5-0.5/2-0, 0]){
        translate([0, 0, u*6/2-u*.375])
        cube([3, 1, u*5.75], center=true);
        translate([0, -length/2, 30-5])
        cube([3, length, 5], center=true);
    }
}

make_top_rear_panel2();

//make_back_button_shim();


//intersection(){
//import("../printable/top_rear_panel_20151122.stl");

//color("blue") translate([0,.3+u*5,0])import("../printable/top_rear_panel_door_20151122.stl");
//}

//translate([0,u*-.25 + u*0, u*0])make_top_rear_panel_door();
