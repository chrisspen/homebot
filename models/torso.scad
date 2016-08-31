include <settings.scad>;

module make_torso_shell(){
    difference(){
        cylinder(h=torso_height, d=max_footprint, center=true);
        cylinder(h=torso_height+1, d=max_footprint-shell_thickness*2, center=true);
        
        translate([0,0,-(torso_height/2+bottom_clearance)+wheel_offset_z])
        hull(){
            rotate([0,90,0])
            cylinder(h=max_footprint+1, d=wheel_diameter+1, center=true);
            
            translate([0,0,-25])
            rotate([0,90,0])
            cylinder(h=max_footprint+1, d=wheel_diameter+1, center=true);
        }
    }
}

make_torso_shell();
