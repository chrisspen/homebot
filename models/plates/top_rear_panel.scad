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

module make_top_rear_panel_door_bb2_hole(tol=0, hole_offset_y=0, rot_offset_z=0){
    
    // must match transformation in make_top_rear_panel_door_bb2
    for(i=[-1:2:1])
    translate([0, 0, (u*5)*i])
    rotate([0, 0, 10+rot_offset_z]){
        color("purple")
        rotate([0, 0, 26])
        rotate([-90, 0, 0])
        translate([0, 0, u*16+hole_offset_y])
        make_countersink(outer=u*2, inner=u*2);
    }
    
}
    
module make_top_rear_panel_door_bb2(tol=0, hole=1, door=0, inner=1, thickness=2, hole_offset_y=0, hinge_offset_y=0){
    
    //angle = 35;//60;
    angle = 60;
    angle_offset0 = tol*2;
    angle_offset = 5 + tol;
    
    height = u*15.5;
    door0_radius = length*2+tol/2-u-thickness;
    
    height2 = u*2;
    door1_radius = length*2+tol/2-u-thickness;
    
    //must be the same?
    r1 = u*.5;
    r2 = u*.5;//1;
    
    length = u*15;
    
    // outer slice
    translate([0,0,0])
    difference(){
        translate([0, 0, -(height - tol)/2])
        rotate([0, 0, -(angle-angle_offset0)/2 + 90])
        rounded_pie(length=length+thickness/2, height=height-tol, r=r1-tol, angle=angle-angle_offset0, fn=100);
        cylinder(d=length*2-thickness, h=height*2, center=true, $fn=50);
    }

    // outer slice tab top
    // must match transformation in make_top_rear_panel_door_bb2_hole
    for(i=[-1:2:1])
    translate([0, 0, (u*5)*i])
    rotate([0, 0, 10]){
        color("green")
        difference(){
            translate([0, 0, -(height2 - tol)/2])
            rotate([0, 0, -(angle-angle_offset0)/2 + 90])
            rounded_pie(length=length+1, height=height2-tol, r=r2-tol, angle=angle-angle_offset0, fn=100);
            cylinder(d=length*2-thickness, h=height2*2, center=true, $fn=50);
        }
    }
    
    if(hole)
    make_top_rear_panel_door_bb2_hole(tol=tol, hole_offset_y=hole_offset_y);
  
    // inner slice
    if(inner)
    color("red")
    translate([0, 0, -(height-u-tol)/2])
    rotate([0, 0, -(angle-angle_offset)/2 + 90])
    rounded_pie(length=length+u, height=height-u-tol, r=u*.5, angle=angle-angle_offset, fn=50);
  
    // hinge tab
    color("blue")
    rotate([0, 0, -29])
    translate([0, u*14.75+.5+hinge_offset_y, u*0])
    cube([u-tol*1, u*.25-tol/2, height-u-tol*2], center=true);
  
}

module make_top_rear_panel_door(height=u*17, thickness=4){
    
    tol = tolerance*2;
    
    difference(){
    
        translate([0,0,u*0])
        rotate([0, 0, -0]){
            make_top_rear_panel_door_bb2(tol=tol, hole=0, inner=0);
        }
     
        // mount hole    
        make_top_rear_panel_door_bb2_hole(tol=tol*0, hole_offset_y=-u*1.15+9.75-9, rot_offset_z=-0.5);
    
    }//end diff
}

module make_top_rear_panel(height=u*17, thickness=4, show_door=1){
        
    difference(){
        union(){
            _make_front_slice_shell(height=height, thickness=thickness);
            
            // mount struts
            intersection(){
                _make_front_slice_bb(height=height, thickness=thickness);
                for(i=[-1:2:1]) for(j=[-1:2:1])
                translate([u*8.5*i, u*11, (-u - height/2 + u*2)*j])
                cube([u, u*6, u*2], center=true);
            }
        }
    
        // mount holes
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([u*8.5*i, u*12+1-5, (height/2 - u)*j])
        rotate([-90, 0, 0])
        make_countersink(inner=20, outer=u*3);
    
        // panel doors
        if(show_door)
        mirror([0,0,0])
        translate([0,0,u*0])
        rotate([0, 0, 0]){
            make_top_rear_panel_door_bb2(thickness=thickness, hinge_offset_y=-0.5);
        }
    
    }//end diff
        
}

//make_top_rear_panel(show_door=1);

//intersection(){
import("../printable/top_rear_panel_20151122.stl");

color("blue") translate([0,.3+u*5,0])import("../printable/top_rear_panel_door_20151122.stl");
//}

//translate([0,u*-.25 + u*0, u*0])make_top_rear_panel_door();
