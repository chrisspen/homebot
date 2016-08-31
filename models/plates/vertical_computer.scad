include <../settings.scad>;
use <plate.scad>;

// This supports the frame vertically and attaches to the horizontal support.
module make_vertical_computer_frame(){
    u = unit_thickness;
    width = vertical_plate_width;
    height = 65;//vertical_plate_height;
    d = screw_thread_diameter;
    
    //top
    translate([0,height/2-u/2,0])
    rotate([0,0,0])
    make_generic_beam(
        u=u,
        a=width/u-2,//actual width
        b=width/u,//width used by holes
        d=d,
        end_fills=0.5,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=1,
        cy2=0
    );
    
    //bottom
    translate([0,-(height/2-u/2),0])
    rotate([0,0,0])
    make_generic_beam(
        u=u,
        a=width/u-2,
        b=width/u,
        d=d,
        end_fills=0.5,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=1,
        cy2=0
    );
    
    //left
    translate([(width/2-u/2),0,0])
    rotate([0,90,0])
    rotate([90,0,90])
    make_generic_beam(
        u=u,
        a=height/u,
        d=d,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=0,
        cy2=1
    );
    
    //right
    translate([-(width/2-u/2),0,0])
    rotate([0,90,0])
    rotate([90,0,90])
    make_generic_beam(
        u=u,
        a=height/u,
        d=d,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=0,
        cx2=0,
        cy1=0,
        cy2=1
    );
    
}

make_vertical_computer_frame();
