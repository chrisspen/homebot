use <../openscad-extra/countersink.scad>;

module make_bnO055_holes(d=2.4){
    //color("blue")
    for(i=[-1:2:1]) for(j=[-1:2:1])
    translate([(27/2-2.4/2-1.3)*i, (21/2-2.4/2-1.3)*j, 0])
    cylinder(d=d, h=20, center=true, $fn=50);
}
    
module make_bnO055(bottom_extra=0){
    
    header_width = 2.54*6;
    
    /*/ header pins
    color("blue")
    translate([0, 16/2+15/2-2.54+5, 2.5/2+2/2+2.5-2.5])
    cube([header_width, 8.88+pin_extra_y, 2.5+.5], center=true);
    */
    
    // header black strip
    //color("red")
    translate([0, 16/2-2.54/2+5, 2.5/2])
    cube([header_width, 2.5, 2.5], center=true);
    
    // header solder through holes
    //color("red")
    translate([0, 21/2 - 2.5/2, -2/2-1.5/2])
    cube([header_width, 2.5, 2+bottom_extra], center=true);
    
    difference(){
        // board
        cube([27,21,1.7], center=true);
        
        // mount holes
        //color("blue")
        make_bnO055_holes();
    }
    
    // smd bb
    translate([0, 0, 2.5/2+1.5/2])
    cube([27, 14, 2.5], center=true);
    
}

module make_bnO055_mount(){
    
    difference(){
        // bounding box
        color([1,0,0,1])
        translate([0, 5/2+1+.5, -5/2])
        cube([30-3, 30-1, 5], center=true);
            
        translate([0,0,1.7/2])
        make_bnO055(bottom_extra=20);
        
        // mount screw cutouts
        color("green")
        for(i=[-1:2:1])
        translate([10*i, 16, 0])
        make_countersink();
        
        // sensor mount holes
        make_bnO055_holes(d=2);
    }
    
    
    
}

//translate([0,0,1.7/2]) make_bnO055();

translate([0,0,0]) make_bnO055_mount();
