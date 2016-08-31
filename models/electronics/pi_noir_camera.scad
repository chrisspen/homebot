
module make_pi_noir_camera_holes(width=25.25, depth=24, height=5, hole_d=2){
    
    translate([width/2-hole_d/2-1,depth/2-hole_d/2-1,0])
    cylinder(d=hole_d, h=height, center=true, $fn=100);
    translate([-(width/2-hole_d/2-1),depth/2-hole_d/2-1,0])
    cylinder(d=hole_d, h=height, center=true, $fn=100);
    
    translate([
        width/2-hole_d/2-1,
        -(depth/2-8.25)+.8,
        0])
    cylinder(d=hole_d, h=height, center=true, $fn=100);
    translate([
        -(width/2-hole_d/2-1),
        -(depth/2-8.25)+.8,
        0])
    cylinder(d=hole_d, h=height, center=true, $fn=100);
    
}

module make_pi_noir_camera(buffer=0){
    width = 25.25;
    depth = 24;

    difference(){
        union(){
    
            // main board
            cube([width,depth,1], center=true);
        
            // camera lense
            translate([0,-depth/2+8/2+5.25,5.5/2+1/2]){
                color("green")
                cube([8+buffer,8+buffer,5.5], center=true);
                color("black")
                cylinder(d=1, h=10, center=true);
            }
        
            // cable jack
            color("green")
            translate([0,-depth/2+6/2,-2.5/2-1/2])
            cube([21,6,2.5], center=true);
    
        }
    
        // mounting holes
        color("blue"){
            make_pi_noir_camera_holes(width=width, depth=depth);
        }
    }
}

make_pi_noir_camera();
