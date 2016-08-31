include <../settings.scad>;
use <plate.scad>;

// This supports the frame horizontally.
module make_horizontal_support(cs_top=0){
    u = unit_thickness;
    width = horizontal_plate_width;
    depth = horizontal_plate_depth;
    d = screw_thread_diameter;

    union(){


    //make_generic_plate(u=u, a=width/u, b=depth/u, d=d, cross=1, border=0);
    
    translate([0,-(width/2+u/2),0])
    rotate([90,0,0])
    make_generic_beam(
        u=u,
        a=width/u,//actual width
        //b=width/u,//width used by holes
        d=d,
        end_fills=1.5,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height,
        cx1=cs_top,
//holes=0,
        cx2=0,
        cy1=0,
        cy2=0
    );

//TODO:incomplete
    
    }
}

make_horizontal_support();
