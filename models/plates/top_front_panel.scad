include <../settings.scad>;
use <countersink.scad>;
use <../rounded_cube.scad>;
use <../electronics/pololu_pushbutton_1439.scad>;

module _make_top_front_bb(width=u*30, height=u*7, thickness=0, h_extend=0, back_extend=0){
    intersection(){
        color("orange")
        cylinder(d=150, h=height+h_extend, center=true, $fn=100);
        
        color("blue")
        translate([0, u*12-back_extend, 0])
        cube([width, u*8 + back_extend*2, height*2+h_extend], center=true);
    }
}

module make_top_front_shell(height){
    difference(){
        color("purple")
        translate([0, -u*13.5 + y_offset, 0])
        _make_top_front_bb(height=height);
        
        // interior rim cutout
        translate([0, -u*13.5 + y_offset, 0])
        cylinder(d=150-u*2, h=u*10+1, center=true, $fn=75);
    }
}

module make_top_front_panel_door_bb(tol=0){
    
        translate([0, u*13.75+5, 0])
        rotate([0, 90, 90])
        rounded_cube_2d([
            u*6-tol,//height
            u*8-tol,//width
            u*2//depth
        ], r=u*.5, $fn=100, center=true);
        
        translate([0, u*13.75, 0])
        rotate([0, 90, 90])
        rounded_cube_2d([
            u*6-u-tol,//height
            u*8-u-tol,//width
            u*2//depth
        ], r=u*.5, $fn=100, center=true);
    
}

module make_top_front_panel_door_bb2_hole(tol=0, offset=0){
    
    // mount hole
    rotate([0, 0, 17.75-.25])
    rotate([-90, 0, 0])
    translate([0,0,u*15.25 - u + u*2 + offset])
    make_countersink(outer=u*2, inner=u*3);
    
}
    
module make_top_front_panel_door_bb2(tol=0, hole=1, door=0, inner=1){
    
    angle = 35;
    angle_offset = 5;
    height = u*6;
    length = u*14.5 + u;
    
    // outer slice
    translate([0,0,0])
    difference(){
        translate([0, 0, -(height - tol)/2])
        rotate([0, 0, -angle/2 + 90])
        rounded_pie(length=length, height=height-tol, r=u*.5, angle=angle, fn=50);
        
        cylinder(d=length*2-u*2+tol/2, h=height*2, center=true, $fn=50);
    }

    // outer slice tab
    color("red")
    translate([0, 0, 0])
    rotate([0, 0, 0])
    difference(){
        translate([0, 0, -(u*2-tol)/2])
        rotate([0, 0, -angle/2 + 90+3.5])
        rounded_pie(length=length, height=u*2-tol, r=u*1-tol, angle=angle, fn=50);
        
        cylinder(d=length*2-u*2, h=height*2, center=true, $fn=50);
    }
    
    // mount hole
    if(hole)
    make_top_front_panel_door_bb2_hole(tol=tol, offset=0);
    
    // inner slice
    if(inner)
    color("red")
    translate([0, 0, -(height-u-tol)/2])
    rotate([0, 0, -(angle-angle_offset)/2 + 90])
    rounded_pie(length=length+u, height=height-u-tol, r=u*.5, angle=angle-angle_offset, fn=50);
    
    // hinge tab
    color("blue")
    rotate([0, 0, -17])
    translate([0, u*14.75, 0])
    cube([u-tol/2, u*.25-tol/2, height-u-tol/2], center=true);
}

module make_top_front_panel(height=u*7){

    sensor_offset_y = 2;
    
    difference(){
        union(){
            make_top_front_shell(height=height);
            
            //bulkhead attaching shell to frame
            for(i=[-1:2:1])
            translate([u*3*i, u*12.5, 0])
            cube([u, u*3, height], center=true);
            
            translate([0, u*13.5, 0])
            cube([u*5, u*2, height], center=true);
        }
        
        // shell mount holes
        color("blue"){
            for(i=[-1:2:1])
            translate([u*3*i, u*14.2-3, -u*2])
            rotate([-90,0,0])
            make_countersink(outer=u*2);
            
            for(i=[-1:2:1])
            translate([u*3*i, u*14.2-3, u*2])
            rotate([-90,0,0])
            make_countersink(outer=u*2);
        }
      
        // panel doors
        for(i=[0:1])
        mirror([i,0,0])
        translate([0,0,u*0])
        rotate([0, 0, -35]){
            make_top_front_panel_door_bb2();
        }
        
        
        // pushbutton cutout
        // https://www.adafruit.com/products/1439
        translate([0,u*17.25-u*2,0])
        rotate([90,0,0])
        rounded_cube_2d([18+tolerance*2, 18+tolerance*2, u*3], r=3+tolerance, $fn=100, center=true);
        translate([0,u*15,0])
        rotate([90,0,0])
        cylinder(d=16+tolerance*2, h=u*10, center=true);
        
    }//end diff
    
    /*
        // small door
        translate([0,0,u*0])
        rotate([0, 0, 0]){
            make_top_front_panel_door_bb2_small();
        }
    */
    
    //_make_shell_hole_supports();
        
}

module make_top_front_panel_door(height=u*7){
    difference(){
        intersection(){
            make_top_front_shell(height=height);
        
            for(i=[0:0])
            mirror([i,0,0])
            translate([0,0,u*0])
            rotate([0, 0, -35]){
                make_top_front_panel_door_bb2(tol=tolerance*2, hole=0, inner=0);
            }
        }
        
        // mount hole
        mirror([1,0,0])
        make_top_front_panel_door_bb2_hole(offset=-u*1.5);
    }
}

module _make_shell_hole_supports(){
    
    a = 5;
    
    color("red")
    for(i=[0:20])
    rotate([0,0,50-a*i])
    translate([0,u*14.5,0])
    if(!(i > 7 && i <= 12))
    cube([.4, u, u*7], center=true);
    
    color("red")
    for(i=[0:2])
    rotate([0,0,10-a*i-5])
    translate([0,u*14.5-u*.5,0])
    cube([.4, u*2, u*7], center=true);
}

make_top_front_panel();
//make_top_front_panel_door();

/*
color("green")
translate([0,u*15,0])
rotate([90,0,0])
make_pololu_pushbutton_1439();
*/

/*
//intersection(){
color("blue")
import("../printable/top_front_panel_20151122.stl");

//translate([0,0,.5])
import("../printable/top_front_panel_door_20151122.stl");
//}
*/
