include <../settings.scad>;
use <countersink.scad>;
use <../rounded_cube.scad>;

module _make_side_shell_bb(width=u*16, height=u*7, thickness=0, h_extend=0, back_extend=0){
    intersection(){
        color("orange")
        cylinder(d=150, h=height+h_extend, center=true, $fn=100);
        
        color("blue")
        translate([0, u*16.5-back_extend, 0])
        cube([width, u*8 + back_extend*2, height*2+h_extend], center=true);
    }
}

module _make_side_shell(height, thickness){
    
    slope_width = u*6;
    
    difference(){
        union(){
            difference(){
                color("purple")
                translate([0, -u*13.5 + y_offset, 0])
                _make_side_shell_bb(height=height);
                
                // interior rim cutout
                translate([0, u*0, 0])
                cylinder(d=u*30-thickness*2, h=height - u*2, center=true, $fn=75);
                
            }//end diff
            
            // bottom screw bulkheads
            intersection(){
                translate([0, -u*13.5 + y_offset, 0])
                _make_side_shell_bb(height=height);
                
                for(i=[-1:2:1])
                translate([u*6*i, u*13, -u*10.5])
                rotate([-90,0,0])
                cylinder(d=u*2, h=u*4, center=true);
            }

            // ramps to close overhangs
            intersection(){
                translate([0, -u*13.5 + y_offset, 0])
                _make_side_shell_bb(height=height);

                union(){
                    translate([0,u*12.5,-u*11.5])
                    rotate([45,0,0])
                    translate([0,0,-slope_width/2])
                    cube([u*20, slope_width, slope_width], center=true);
                
                    translate([0,u*12.5,u*11.5])
                    rotate([-45,0,0])
                    translate([0,0,slope_width/2])
                    cube([u*20, slope_width, slope_width], center=true);
                }
            }
        }
        
        // top screws
        for(i=[-1:2:1])
        translate([u*5*i, u*13.75, u*12])
        rotate([-90,0,0])
        make_countersink(inner=u*2, outer=u*2);
        
        // bottom screws
        for(i=[-1:2:1])
        translate([u*6*i, u*13.35, -u*10.5])
        rotate([-90,0,0])
        make_countersink(inner=u*2, outer=u*2);
    }
}

module make_side_panel(){
    _make_side_shell(height=u*25, thickness=u*.5);
}

make_side_panel();
