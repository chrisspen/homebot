
use <countersink.scad>;

module make_head_wire_through($fn=100){
    
    y_offset_center = 22.5-5*4;
    
    height = 5*2+0.5*2;
    
    d = 5*5;
    
    difference(){
        union(){
            // outer bb
            cylinder(d=d - 0.5*2, h=height, center=true);
            
            // centering notch
            color("red")
            translate([0, 5*2.5, 5/2+0.5 + 3/2])
            rotate([0,0,45])
            cube([2,2,5-3], center=true);
            
            // rims
            for(i=[-1:2:1])
            translate([0,0, (height/2 + .5)*i])
            cylinder(d=d + 2, h=1, center=true);
        }
        
        difference(){
            // inner bb
            cylinder(d=d - 0.5*2 - 1*2, h=5*5, center=true);
            
            color("red")
            translate([0,-100/2 - 7,0])
            cube([100, 100, 100], center=true);
        }
        
        color("blue")
        translate([0,-10,4.5+1])
        make_countersink();
    }

}

module _make_head_wire_through_cutter_a($fn=100){
    translate([0,0,-100/2])
    cube([100, 100, 100], center=true);
    cylinder(d=5*5 - 0.5*2 - 1, h=2, center=true);
}

module _make_head_wire_through_cutter_b($fn=100){
    difference(){
        translate([0,0,100/2])
        cube([100, 100, 100], center=true);
        translate([0,0,0])
        cylinder(d=5*5 - 0.5*2 - 1 - 0.25, h=2, center=true);
    }
}

module make_head_wire_through_a(){
    difference(){
        make_head_wire_through();

        color("red")
        //translate([0,0,-100/2 + 3])cube([100, 100, 100], center=true);
        translate([0,0,3]) _make_head_wire_through_cutter_a();
    }
    
}

module make_head_wire_through_b(){
    difference(){
        make_head_wire_through();

        color("red")
        //translate([0,0,100/2 + 3]) cube([100, 100, 100], center=true);
        translate([0,0,3]) _make_head_wire_through_cutter_b();
    }
}

//make_head_wire_through();

//_make_head_wire_through_cutter_a();

//_make_head_wire_through_cutter_b();

make_head_wire_through_a();

translate([0,40,0]) rotate([0,180,0]) make_head_wire_through_b();

/*/ overlap test
intersection(){
make_head_wire_through_a();
translate([0,0,-.05])
make_head_wire_through_b();
}
*/

/*/ overlap test
intersection(){
union(){
    make_head_wire_through_a();
    make_head_wire_through_b();
}

union(){
    translate([0,0,-2.5])
    rotate([0,90,0])
    import("../printable/head_strut_open_side_20160601.stl");

    color("blue")
    translate([0,-22.5,2.5])
    import("../printable/neck_strut_open_20160601.stl");
}
}*/