include <../settings.scad>;
use <countersink.scad>;

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
        translate([0,0,thickness])
        cylinder(d=150-thickness*2, h=height, center=true, $fn=75);
            
    }
    
}

module make_lower_rear_panel(){
    
    height = u*8;
    thickness = 2;
    
    difference(){
        union(){
            difference(){
                _make_front_slice_shell(height=height);
                
                // tread cutouts
                for(i=[-1:2:1])
                translate([(u*10.25)*i, u*9, -u*4])
                cube([u*2.5,u*8,u*4], center=true);
            }
            
            intersection(){
                _make_front_slice_bb(height=height, thickness=thickness);
                
                // mount struts
                for(i=[-1:2:1]) for(j=[-1:2:1])
                translate([(u*7.5)*i, u*11, (-u - height/2 + u*2)*j])
                cube([u, u*6, u*2], center=true);
            }
        }
        
        // dc converter plate cutout
        translate([0, u*7.5 + 3, -height/2 + thickness/2])
        cube([u*14+tolerance, u, thickness+tolerance], center=true);
    
        // mount holes
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([u*7.5*i, u*12+1, (height/2 - u)*j])
        rotate([-90, 0, 0])
        make_countersink(inner=25, outer=u*3);
        
    }
}

make_lower_rear_panel();
