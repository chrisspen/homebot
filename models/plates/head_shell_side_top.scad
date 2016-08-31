use <head_shell.scad>;

translate([-30,-30,-87])
difference(){
    translate([0,0,87])
    rotate([0,0,45])
    make_head_shell_side();

    translate([0,0,-150/2+87.5-0.5])
    cube([200,200,150], center=true);
}

/*
color("gray")
translate([150,0,150/2])
cube([150,150,150], center=true);
*/