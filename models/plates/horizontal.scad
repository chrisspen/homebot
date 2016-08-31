include <../settings.scad>;
use <plate.scad>;

// This supports the frame horizontally.
module make_horizontal_frame(cs_top=0, skip_i_a=-1, skip_i_b=-1, skip_i_c=-1, skip_i_d=-1, width=0, depth=0){
    u = unit_thickness;
    width = (width) ? width : horizontal_plate_width;
    depth = (depth) ? depth : horizontal_plate_depth;
    d = screw_thread_diameter;

    union(){
    
        /*
        translate([0,0,-10]){
            make_generic_plate(u=u, a=width/u, b=depth/u, d=d, cross=0, border=1);
        }
        */
    
        make_generic_plate(u=u, a=width/u, b=depth/u, d=d, cross=1, border=0);
        
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
            skip_i=skip_i_a,
            cx2=0,
            cy1=0,
            cy2=0
        );
        
        translate([0,(width/2+u/2),0])
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
            skip_i=skip_i_b,
            cx2=0,
            cy1=0,
            cy2=0
        );
        
        translate([-(width/2-u/2),0,0])
        rotate([90,0,90])
        make_generic_beam(
            u=u,
            a=depth/u,//actual width
            //b=width/u,//width used by holes
            d=d,
            end_fills=1.5,
            countersink_d=screw_head_diameter,
            countersink_d2=screw_thread_diameter,
            countersink_depth=screw_head_height,
            cx1=cs_top,
            skip_i=skip_i_c,
            cx2=0,
            cy1=0,
            cy2=0
        );
        
        translate([(width/2-u/2),0,0])
        rotate([90,0,90])
        make_generic_beam(
            u=u,
            a=depth/u,//actual width
            //b=width/u,//width used by holes
            d=d,
            end_fills=1.5,
            countersink_d=screw_head_diameter,
            countersink_d2=screw_thread_diameter,
            countersink_depth=screw_head_height,
            cx1=cs_top,
            skip_i=skip_i_d,
            cx2=0,
            cy1=0,
            cy2=0
        );
    
    }
}

make_horizontal_frame();
