include <../settings.scad>;
use <plate.scad>;
use <../plates/countersink.scad>;

module make_conduit_beam_side(x_units=3, z_units=24, holes=1){
    
    n_holes = z_units/6;
    m_holes = z_units;
    
    difference(){
        cube([u, u*x_units, u*z_units], center=true);
    
        if(holes){
            color("red"){
                for(i=[0:1:m_holes-1]){
                    translate([0,-u*1.5,-u*z_units/2 + u/2 + u*i])
                    rotate([90,0,0])
                    cylinder(h=u*2.1, d=screw_thread_diameter, center=true);
                }
            }
            color("red"){
                for(i=[0:1:n_holes]){
                    translate([0,0,-u*z_units/2 + u*3 + u*i*4.75])
                    scale([1,1,3.5])
                    rotate([0, 90, 0])
                    cylinder(h=u*1.1, d=u, center=true);
                }
            }
        }
    }//end diff
}

module make_conduit_beam_middle(x_units=3, y_units=6, holes=1){
    middle_hole_d = (y_units-1)*u;
    
    difference(){
        union(){
            difference(){
                cube([u*y_units*.9, u*x_units, u], center=true);
                
                color("blue")
                translate([0,-middle_hole_d/2+u*x_units/2-u,0])
                cylinder(d=middle_hole_d, h=u*2, center=true);
                
                if(holes)
                color("red"){
                    translate([u,u,0])
                    rotate([90,0,0])
                    make_countersink();
                    translate([-u,u,0])
                    rotate([90,0,0])
                    make_countersink();
                }
            }
            
            translate([u*y_units,0,0])
            difference(){
                cube([u*y_units*.9, u*x_units, u], center=true);
                
                color("blue")
                translate([0,-middle_hole_d/2+u*x_units/2-u,0])
                cylinder(d=middle_hole_d, h=u*2, center=true);
                
                if(holes)
                color("red"){
                    translate([-u,u,0])
                    rotate([90,0,0])
                    make_countersink();
                }
                
                translate([u*y_units*.25,0,0])
                cube([u*y_units/2,u*x_units*2,u*1.1], center=true);
            }
            mirror([1,0,0])
            translate([u*y_units,0,0])
            difference(){
                cube([u*y_units*.9, u*x_units, u], center=true);
                
                color("blue")
                translate([0,-middle_hole_d/2+u*x_units/2-u,0])
                cylinder(d=middle_hole_d, h=u*2, center=true);
                
                if(holes)
                color("red"){
                    translate([-u,u,0])
                    rotate([90,0,0])
                    make_countersink();
                }
                
                translate([u*y_units*.25,0,0])
                cube([u*y_units/2,u*x_units*2,u*1.1], center=true);
            }
            
            difference(){
                cube([u*y_units*.9, u*x_units, u], center=true);
                
                color("blue")
                translate([0,-middle_hole_d/2+u*x_units/2-u,0])
                cylinder(d=middle_hole_d, h=u*2, center=true);
                
                if(holes)
                color("red"){
                    translate([u,u,0])
                    rotate([90,0,0])
                    make_countersink();
                    translate([-u,u,0])
                    rotate([90,0,0])
                    make_countersink();
                }
            }
        }
            
        // bottom mounting holes
        color("green")
        for(i=[0:4])
        translate([-u*4 + u*2*i,u,u])
        rotate([0,0,0])
        make_countersink(inner=u*2, outer=0);
    }
}

module make_conduit_beam(x_units=3, y_units=6, z_units=25, holes=1){

    n_holes = z_units/5;
    
    translate([u*y_units/2,0,0])
    make_conduit_beam_side(x_units=x_units, z_units=z_units, holes=holes);
    
    translate([-u*y_units/2,0,0])
    make_conduit_beam_side(x_units=x_units, z_units=z_units, holes=holes);
    
    translate([0,0,u*z_units/2-u/2])
    make_conduit_beam_middle(x_units=x_units, y_units=y_units, holes=holes);
        
    translate([0,0,-(u*z_units/2-u/2)])
    make_conduit_beam_middle(x_units=x_units, y_units=y_units, holes=holes);
    
}

rotate([-90, 0, 0])
make_conduit_beam(holes=1);
