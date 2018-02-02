/*
Countersink depression for screw heads.
*/
include <../settings.scad>;
use <../openscad-extra/src/countersink.scad>;

/*
module make_countersink(d1=0, d2=0, h=0, l=0, outer=100, inner=200, bt_extra=0, $fn=25){
    extra = 0.5;
    d1 = d1 ? d1 : screw_thread_diameter;
    d2 = d2 ? d2 : screw_head_diameter;
    h = h ? h : screw_head_height;
    
    union(){
        cylinder(d1=d1, d2=d2, h=h+extra, center=true, $fn=$fn);
        
        if(l)
        cylinder(d=d1, h=l, center=true, $fn=$fn);
    
        // big top
        translate([0,0,(h+extra)/2])
        cylinder(d=d2+bt_extra, h=outer, center=false, $fn=$fn);

        // lower bottom
        translate([0,0,-(h+extra)/2-inner])
        cylinder(d=d1, h=inner, center=false, $fn=$fn);
    }
}
*/

module make_countersink_motor(){
    make_countersink(
        d1=motor_mount_holes_width,
        d2=motor_mount_holes_head_width,
        h=motor_mount_holes_head_height);
}

make_countersink();
