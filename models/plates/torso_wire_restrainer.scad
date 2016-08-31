
use <../openscad-extra/countersink.scad>;
use <../openscad-extra/rounded.scad>;

module make_torso_wire_restrainer(){
    
    //depth=50
    //width=75
    //c=sqrt(a2+b2)
    depth = 50;
    width = 75;
    diagonal_length = sqrt(depth*depth + width*width) + 5;
    angle = atan(width/depth);
    
    difference(){
    
        union(){
            for(i=[-1:2:1])
            rotate([0,0,angle*i])
            translate([0,0,0])
            rounded_cube_2d([5, diagonal_length, 5], r=1, $fn=100, center=true);
            
            translate([0,-15-5,0])
            rounded_cube_2d([5, 40, 5], r=1, $fn=100, center=true);
        }
        
        //mock for mount points
        //color("blue")cube([width,depth,1], center=true);
        
        color("green"){
            //main mount holes
            for(i=[-1:2:1]) for(j=[-1:2:1])
            translate([width/2*i,depth/2*j,3.9])
            make_countersink(d1=2.5+0.5, d2=5);
            
            //extra fifth mount hole
            translate([0,-35-2.5,3.9])
            make_countersink(d1=2.5+0.5, d2=5);
        }
        
        // mass cutout to give wires room
        color("red")
        translate([0,0,-5/2-1.25+5-1])
        cube([70,70,5], center=true);
        
    }
}

rotate([0, 180, 0])
make_torso_wire_restrainer();
