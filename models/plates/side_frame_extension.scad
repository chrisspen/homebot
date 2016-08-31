include <../settings.scad>;
use <cross_plate.scad>;
use <countersink.scad>;

module make_side_frame_extension(){
    h = 17;
    w = 4.5;
    
    difference(){
          
        /*
        difference(){
            cube([u*w, u, u*h], center=true);
            cube([u*w-u*2, u*2, u*h-u*2], center=true);
        }
        */ 
        rotate([0,0,90])
        make_cross_plate(
            w=u*w,
            h=u*h,
            hub_d=0,//u*4,
            t=u,
            cross_type=1
        );
        
        // general mount holes through width
        color("red")
        for(i=[0:h-4])
        translate([0, 0, -h*u/2 + u*2 + u*i])
        rotate([0,-90,0])
        cylinder(d=screw_thread_diameter, h=u*w*1.1, center=true, $fn=25);
        
        // general mount holes through thickness
        color("red")
        for(i=[0:h-2]) for(j=[-1:2:1]) if(i!=3 && i!=11)
        translate([(w*u/2 - u/2)*j, 0, -h*u/2 + u*1 + u*i])
        rotate([0,-90,90])
        cylinder(d=screw_thread_diameter, h=u*w*1.1, center=true, $fn=25);
        
        // frame mount holes
        translate([-w*u*.45 + .2, 0, -h*u/2 + u*4])
        rotate([0, -90, 0])
        make_countersink(inner=u*5 - 2, $fn=25);
        translate([-w*u*.45 + .2, 0, h*u/2 - u*5])
        rotate([0, -90, 0])
        make_countersink(inner=u*5 - 2, $fn=25);
     
        // top/bottom mount holes
        for(i=[0:2]) for(j=[-1:2:1])
        color("red")
        translate([-w*u*.4 + u*.5 + u*i, u*.3, (h*u/2 - u*.5)*j])
        rotate([-90, 0, 0])
        make_countersink(inner=u*5 - 2, $fn=25);
        
    }

}

rotate([90, 0, 0])
make_side_frame_extension();
