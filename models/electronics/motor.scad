include <../settings.scad>;

module make_motor(flip=0){
    rotate([0,90+(180*flip),0]){
        
        difference(){
            cylinder(h=motor_length, d=motor_width, center=true);
            
            translate([motor_mount_holes_dist/2,0,motor_length/2])
            cylinder(h=motor_length, d=motor_mount_holes_width, center=true);
            
            translate([-motor_mount_holes_dist/2,0,motor_length/2])
            cylinder(h=motor_length, d=motor_mount_holes_width, center=true);
        }
        
        translate([0,0,motor_collar_height/2+motor_length/2])
        cylinder(h=motor_collar_height, d=motor_collar_width, center=true);
        
        translate([0,0,motor_axle_height/2+motor_collar_height/2+motor_length/2])
        cylinder(h=motor_axle_height, d=motor_axle_width, center=true);
    }
}

module make_pololu_motor_163F3F_axle(d=3, h=9.9){
    notch_thickness = 2.46;
    difference(){
        color("red")
        cylinder(d=d, h=h);
        color("blue")
        translate([0,notch_thickness,0])
        cube([d,d,d*h], center=true);
    }
}

module make_pololu_motor_163F3F(){
    //https://www.pololu.com/product/2361
    //24.25
    //25.5
    gearbox_length = 9;
    housing_length = 15.25;
    housing_d = 12.05;
    union(){
    
        // motor housing
        translate([0,housing_length/2-1/2,0])
        intersection(){
            cube([12, housing_length+1, 10], center=true);
            rotate([90,0,0])
            cylinder(d=housing_d, h=housing_length+1, center=true);
        }
        translate([0,housing_length+1.6,0])
        rotate([90,0,0])
        cylinder(d=5, h=1.6);
    
        // gear box
        translate([0,-gearbox_length/2,0])
        cube([12, gearbox_length, 10], center=true);
        
        // axle collar
        translate([0,-gearbox_length,0])
        rotate([90,0,0])
        cylinder(d=4, h=0.5);
        
        // axle
        translate([0,-gearbox_length,0])
        rotate([90,0,0])
        //cylinder(d=3, h=9.9);
        make_pololu_motor_163F3F_axle();
    }
}

//make_motor();
make_pololu_motor_163F3F();
