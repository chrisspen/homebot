include <../settings.scad>;

module make_axle_a(){

    h = track_wheel_width/2 - 0.2;

    difference(){
        union(){
            // main body
            // this should be slightly smaller than the hole on the idler wheel
            cylinder(
                d=track_wheel_hole_radius*2-2,
                h=h-motor_bearing_thickness);
            
            // nub sticking out to center the bearing
            // this should be slightly smaller than the hole on the bearing
            cylinder(d=track_axle_bearing_diameter-0.2, h=h);
        }
        // axle hole
        cylinder(
            d=screw_thread_diameter+tolerance*1.5,
            h=track_wheel_gap*2, center=true);
    }
}

module make_axle_b(){

    h = track_wheel_width_buffer;

    difference(){
        union(){
            // main body
            // this should be slightly larger than the hole on the idler wheel
            cylinder(
                d=track_wheel_hole_radius*2+2,
                h=h/2);
            
            // nub sticking out to center the bearing
            // this should be slightly smaller than the hole on the idler wheel
            cylinder(
                d=track_wheel_hole_radius*2-2,
                h=h);
        }
        // axle hole
        cylinder(
            d=screw_thread_diameter+tolerance*1.5,
            h=track_wheel_gap*2, center=true);
    }
}


make_axle_a();

//translate([0,15,0]) make_axle_b();
