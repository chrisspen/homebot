use <countersink.scad>;

module make_mpu6050(pin_extra_y=0, smd_extra_z=0){
    
    // header pins
    color("blue")
    translate([0, 16/2+(8.88+pin_extra_y)/2-2.54, 2.5/2+2/2+2.5])
    cube([21+0.5, 8.88+pin_extra_y, 2.5+.5], center=true);
    
    // header black strip
    color("red")
    translate([0, 16/2-2.54/2, 2.5/2+2/2])
    cube([21, 2.54+0.5, 2.5], center=true);
    
    // header solder through holes
    color("red")
    translate([0, 16/2-2.54/2, -(1.5/2+2/2)])
    cube([21, 2.54+0.5, 1.5], center=true);
    
    // board
    cube([21,16,2], center=true);
    
    // smd bb
    translate([0,-2,(2+smd_extra_z)/2 + 2/2])
    cube([21,10,2+smd_extra_z], center=true);
    
}


module make_mpu6050_mount(){
    
    difference(){
        
        // bounding box
        color([1,0,0,1])
        translate([0, -.5+6, 2])
        cube([25, 30, 10], center=true);
        
        // screw lip cutout
        color("green")
        translate([0, 30/2 + 3, 20/2-3+5])
        cube([30,5+0.5,20], center=true);
    
        // board cutout
        make_mpu6050(pin_extra_y=10, smd_extra_z=10);
        translate([5,0,0])
        make_mpu6050(pin_extra_y=0, smd_extra_z=0);
        
        // mount screw cutouts
        for(i=[-1:2:1])
        //translate([(25/2 - 5/2)*i, 30/2 -.5+6 - 5/2, 1])
        translate([(10)*i, 30/2 -.5+6 - 5/2, 1])
        make_countersink();
    }
}

//make_mpu6050(pin_extra_y=0, smd_extra_z=0);
rotate([0,-90,0])
make_mpu6050_mount();
