include <../settings.scad>;
use <../rounded_cube.scad>;
use <countersink.scad>;

module make_screw_tab(block_size=5){
    translate([0, 0, 0])
    rounded_cube([block_size, block_size, block_size], center=true);
}

module make_screw_terminal(number=4, block_size=5, hole_d=2, wire_d=1.5, show_tabs=1, rail_d=1, $fn=50){
    
    divider_height = u;
    
    difference(){
        
        // main bulkhead
        union(){
            translate([0,0,u*0.25*.5])
            cube([block_size*2, block_size*number, block_size*1.25], center=true);
            
            translate([0, 0, divider_height/2])
            cube([0.5, block_size*number, block_size+divider_height], center=true);
        }
        
        for(i=[0:number-1])
        for(j=[-1:2:1])
        color("red"){
        
            // top screw holes    
            translate([block_size/2*j, -(block_size*number)/2 + block_size/2 + block_size*i, block_size*.3])
            cylinder(d=hole_d, h=block_size*1.5, center=true, $fn=$fn);
            
            // hookup wire holes
            translate([block_size*j, -(block_size*number)/2 + block_size/2 + block_size*i, 0])
            rotate([0, 90, 0])
            cylinder(d=wire_d, h=block_size*.95, center=true, $fn=$fn);
            
            // rail wire holes
            translate([block_size/2*j, 0, -block_size*.25])
            rotate([90, 0, 0])
            cylinder(d=rail_d, h=block_size*number*2, center=true, $fn=$fn);
        }
        
    }//end diff
    
    if(show_tabs){
        translate([block_size/2, block_size*number*.5+block_size*.5, 0]){
            difference(){
                make_screw_tab(block_size=block_size*1);
                translate([0, 0, 1.5])
                make_countersink();
            }
        }
        
        translate([-block_size/2, -(block_size*number*.5+block_size*.5), 0]){
            difference(){
                make_screw_tab(block_size=block_size*1);
                translate([0, 0, 1.5])
                make_countersink();
            }
        }
    }
}

make_screw_terminal();
