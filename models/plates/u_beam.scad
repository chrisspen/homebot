include <../settings.scad>;
use <plate.scad>;
use <../plates/countersink.scad>;

module make_u_beam(x_units=4, y_units=4){

    y_offset = (y_units - 4)*u;

    difference(){
        union(){
    	    cube([u, u*y_units, u], center=true);
    	
    	    translate([u*x_units/2-u/2, u*y_units/2, 0])
    	    rotate([0, 0, 90])
    	    cube([u, u*x_units, u], center=true);
    	
    	    translate([u*x_units/2-u/2, -u*y_units/2, 0])
    	    rotate([0, 0, 90])
    	    cube([u, u*x_units, u], center=true);
        }
    
        color("red"){
        
            // back
            translate([u*0.3, u, 0])
            rotate([0, 90, 0])
            make_countersink();
            translate([u*0.3, -u, 0])
            rotate([0, 90, 0])
            make_countersink();
    
            // left
            translate([u*0.5, u*4 + y_offset/2, 0])
            rotate([0, 90, 90])
            make_countersink(inner=u*2.31);
            translate([u*2.5, u*4 + y_offset/2, 0])
            rotate([0, 90, 90])
            make_countersink(inner=u*2.31);
    
            // right
            translate([u*0.5, -u*4 - y_offset/2, 0])
            rotate([0, 90, -90])
            make_countersink(inner=u*2.31);
    
            translate([u*2.5, -u*4 - y_offset/2, 0])
            rotate([0, 90, -90])
            make_countersink(inner=u*2.31);
    
        }
    }
}

module make_back_beam(x_units=4, y_units=4, holes=1){

    y_offset = (y_units - 4)*u;

    difference(){
        union(){
            cube([u, u*y_units, u], center=true);
        
        }
    
        if(holes)
        color("red"){
        
            // back
            translate([u*0.3, u, 0])
            rotate([0, 90, 0])
            make_countersink();
            translate([u*0.3, -u, 0])
            rotate([0, 90, 0])
            make_countersink();
    
            // left
            translate([u*0.0, u*4 + y_offset/2, 0])
            rotate([0, 90, 90])
            make_countersink(inner=u*2.8);
    
            // right
            translate([u*0.0, -u*4 - y_offset/2, 0])
            rotate([0, 90, -90])
            make_countersink(inner=u*2.8);
    
    
        }
    }
}

module make_conduit_beam(x_units=4, z_units=10, holes=1){

        union(){
            translate([0,0,z_units*u/2])
            cube([u, u*x_units, u], center=true);
            translate([0,0,-z_units*u/2])
            cube([u, u*x_units, u], center=true);
            
            translate([0,u*x_units/2-u/2,0])
            cube([u, u, u*z_units], center=true);
            translate([0,-u*x_units/2+u/2,0])
            cube([u, u, u*z_units], center=true);
        
        }    
}

//make_u_beam();
make_conduit_beam();
