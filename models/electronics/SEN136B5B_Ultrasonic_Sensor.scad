/*
http://www.seeedstudio.com/wiki/index.php?title=Ultra_Sonic_range_measurement_module
*/

module make_ultrasonic_sensor_mount_holes(width, length, hole_diameter, hole_brim, height=50){

    for(i=[1,-1]){ for(j=[1,-1]){
        translate([i*(width/2-hole_diameter/2-hole_brim), j*(length/2-hole_diameter/2-hole_brim), 0]){
            color("red")
            cylinder(h=height, d=hole_diameter, center=true);
            
        }
    }}
}

module make_ultrasonic_sensor(cylinder_extend=0, cylinder_tol=0, show_mounts=0, mount_height=20, smd_tol=0){

    $fn = 100;

    width = 21;
    length = 45.15;
    thickness = 1;

    hole_diameter = 2;
    hole_brim = 1;
    
    cylinder_d_base = 16;
    cylinder_d = cylinder_d_base + cylinder_tol;
    
    cylinder_offset = (8.5-.8)/2 + cylinder_d_base/2;

    difference(){
        minkowski(){
            cube([width-2, length-2, thickness/2], center=true);
            cylinder(h=thickness/2, d=2, center=true);
        }
        
        for(i=[1,-1]){ for(j=[1,-1]){
            translate([i*(width/2-hole_diameter/2-hole_brim), j*(length/2-hole_diameter/2-hole_brim), 0]){
                color("red")
                cylinder(h=5, d=hole_diameter, center=true);
                
            }
        }}
    }//end diff

    if(show_mounts)
        make_ultrasonic_sensor_mount_holes(width, length, hole_diameter, hole_brim, height=mount_height);
    
    
    color("green"){
        translate([0, cylinder_offset, .5])
        cylinder(h=12+cylinder_extend, d=cylinder_d, center=false);
     
        translate([0, -cylinder_offset, .5])
        cylinder(h=12+cylinder_extend, d=cylinder_d, center=false);
     
    }

    color("blue")
    hull()
    for(i=[-1:2:1])
    translate([7.5*i,0,2.0])
    hull(){
        translate([0,5.5-2,0])
        cylinder(h=3+smd_tol, d=4+smd_tol, center=true);
    
        translate([0,-(5.5-2),0])
        cylinder(h=3+smd_tol, d=4+smd_tol, center=true);
    }

    translate([-9.5,0,-1.5]){
        color("purple")
        cube([2,8,2], center=true);
    }

}

make_ultrasonic_sensor(show_mounts=1, smd_tol=2);
