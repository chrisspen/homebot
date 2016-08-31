include <../settings.scad>;
use <flexiTreads.scad>;

module make_track_idler(){
	hole_d = 10+0.3;
    if(track_wheel_show_fancy){
        difference(){
            union(){
                wheel(track_wheel_radius, track_wheel_width, track_wheel_offset, false);
                
                color("red")
                for(i=[0:1])
                translate([0,0,-0.5 + i*11])
                mirror([0,0,i])
                cylinder(d1=track_wheel_radius*2+2, d2=track_wheel_radius*2-4, h=1, center=true, $fn=100);
            }
                    
            // axle hole
            cylinder(
                d=screw_thread_diameter+tolerance*1.5,
                h=track_wheel_gap*2, center=true);
        
            // top bearing
            translate([0, 0, 13.5 - 2.5])
            cylinder(d=10+0.25, h=2.5*2, center=true);
        
            // bottom bearing
            translate([0, 0, -3.5+2.5])
            cylinder(d=10+0.25, h=2.5*2, center=true);
            
        }
    }else{
        cylinder(d=track_wheel_radius*2, h=track_wheel_width, center=false);
        cylinder(d=hole_d, h=track_wheel_width*10, center=true);
    }
}

make_track_idler();
