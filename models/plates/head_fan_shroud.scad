include <../settings.scad>;

use <../electronics/dfrobot_rpi_arduino_hat.scad>;
use <../openscad-extra/countersink.scad>;

module make_square_pipe_mount(w=10, h=10, d=5, length=5, thickness=1.5){
    
    top_offset_x = -1.75;
    top_offset_y = -2.5;
    
    difference(){
        hull(){
            translate([top_offset_x, top_offset_y, length])
            rotate([0,0,0])
            cylinder(d=d, h=.1, center=true, $fn=50);

            cube([w,h,.1],center=true);
        }
        
        hull(){
            translate([top_offset_x, top_offset_y, length+.1])
            rotate([0,0,0])
            cylinder(d=d-thickness*2, h=.1, center=true, $fn=50);

            translate([0,0,-0.1])
            cube([w-thickness*2,h-thickness*2,.1],center=true);
        }
    }
}

module _make_pipe_split(d1, length, thickness=0, length_offset=0){
    
    offset_up = -1;
    
    scale_width = 0.9;
    
    scale_height = 0.4;
    
    bottom_offset_x = -2;

    union(){
        // bottom pipe
        hull(){
            translate([bottom_offset_x, 2+offset_up, length+length_offset])
            scale([1,scale_height,1])
            cylinder(d=d1*scale_width-thickness*2, h=0.01, center=true, $fn=50);

            cylinder(d=d1-thickness*2, h=0.01, center=true, $fn=50);
        }
        // top pipe
        hull(){
            translate([bottom_offset_x, -5+offset_up, length+length_offset])
            scale([1,scale_height,1])
            cylinder(d=d1*scale_width-thickness*2, h=0.01, center=true, $fn=50);

            cylinder(d=d1-thickness*2, h=0.01, center=true, $fn=50);
        }
    }

}

module make_pipe_split(d1=17, length=2, thickness=1.5){
    
    difference(){
        
        // main bulk
        _make_pipe_split(d1, length, thickness=0);
        
        // inner cutout
        color("red")
        _make_pipe_split(d1, length, thickness=thickness);
    
    }
}

module make_pipe_segment(d1=17, length=1, thickness=1){
    
    scale_height = 0.4;
    
    scale([1, scale_height, 1])
    difference(){
        cylinder(d=d1, h=length, center=true, $fn=50);
        
        cylinder(d=d1-thickness*2, h=length+0.1, center=true, $fn=50);
    }
    
}

module make_pipe_curved_segment(point1, point2, d, offset=0){
    hull(){
        translate(point1)
        rotate([90,0,0])
        scale([1+offset,1+offset,1+offset])
        cylinder(d=d, h=.01, center=true, $fn=50);
        
        translate(point2)
        rotate([90,0,0])
        scale([1+offset,1+offset,1+offset])
        cylinder(d=d, h=.01, center=true, $fn=50);
    }
}

module make_pipe_curved(points, inner=1, outer=2){
    length = len(points);
    echo(length);
    for(i=[0:len(points)-2]){
        difference(){
            make_pipe_curved_segment(point1=points[i], point2=points[i+1], d=outer);
            make_pipe_curved_segment(point1=points[i], point2=points[i+1], d=inner, offset=.1);
        }
    }
}

module make_head_fan_shroud(){
    
    housing_offset_y = 4.5;
    housing_offset_z = 0.25;
    
    difference(){
        union(){
            // mount bar
            cube([75, 5, 5], center=true);
            
            // main fan housing body
            translate([0, -1.5-0.5/2 + housing_offset_y, -1-0.5/2 + housing_offset_z])
            cube([17.5+2, 8.5+2, 17.5+2], center=true);
        }
        
        translate([0,housing_offset_y,housing_offset_z])
        union(){
            // fan bb, extended along y
            translate([0, -1.5-0.5/2, -1-0.5/2])
            cube([17, 8.5+10, 17], center=true);
            
            // fan bb, extended along z
            translate([0, -1.5-0.5/2, -1-0.5/2+10/2])
            cube([17.5, 8.5, 17.5+10], center=true);
        }
        
        // mount holes
        for(i=[-1:2:1])
        color("red")
        translate([u*7*i,-2.75,0])
        rotate([90,0,0])
        make_countersink(d1=3, d2=5);
    
        translate([0,1.25,10])
        cube([75, 4, 5], center=true);    
        
    }// end diff

    translate([0,u*10.5,+u*6]){
    
        if(1)
        translate([0,-44.5,-31])
        rotate([-90,0,0])
        make_square_pipe_mount(w=17.5+2, h=17.5+2, d=17, length=2.5);

        if(1)
        translate([-1.75,-41.95,-31+2.5])
        rotate([-90,0,0])
        make_pipe_split();

        if(1)
        translate([-3.75,-39+1.4,-29.5])
        rotate([-90,0,0])
        make_pipe_segment(d1=15, length=4.75, thickness=1.25);

        if(1)
        translate([-3.75,-39.95,-22.85+0.25])
        scale([.9,1,0.375])
        make_pipe_curved(
            points=[
                [0,0,0],
                [0,5,13],
                [-2,15,13],
                [-4,24,-2]
            ], inner=17-4, outer=17);

    }
}

if(1)
rotate([90,0,0])
translate([0,-u*10.5,-u*6])
make_head_fan_shroud();

if(0){
    
    if(1)
    color("orange")
    translate([0, 0, 0])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0, 0, 30+u/2-65-u/2])
    rotate([0,0,0])
    //make_head_top();
    import("../printable/head_bottom.stl");

    translate([-1.75,0,-2.5]){
        color("purple")
        translate([0, 0, 0])
        rotate([head_vertical_angle_tilt,0,0])
        translate([-30, 50, -30+u])
        rotate([0,0,-90])
        import("../electronics/RPi2_01.stl");
        
        if(1)
        color("MediumVioletRed")
        rotate([head_vertical_angle_tilt,0,0])
        translate([0,0,-7])
        make_dfrobot_rpi_arduino_hat();
    }

    if(1)
    color("orange")
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-u*10.5,0])
    rotate([0,-90,90])
    import("../printable/head_face_sensor_mount_20160214.stl");

    if(1)
    color("green")
    rotate([head_vertical_angle_tilt,0,0])
    translate([0, -(-u*.5-tolerance/2), 38 - 65-u])
    //rpi_tray();
    import("../printable/rpi_tray2.stl");

    color("lightgreen")
    rotate([head_vertical_angle_tilt,0,0])
    translate([0, -50+u/2, 0])
    rotate([0,-90,90])
    import("../printable/head_front.stl");
    
    if(0){
        translate([0,0,0])
        rotate([head_vertical_angle_tilt,0,0])
        //make_head_shell_front();
        import("../printable/head_shell_front.stl");
    }
}
