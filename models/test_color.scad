/*
F6 rendering not showing color in OpenSCAD version 2015.03-1
openscad --autocenter --projection=ortho --preview --viewall -o test.png test_color.scad
openscad --autocenter --projection=ortho --preview --camera=50,50,50,0,0,0 -o test.png test_color.scad
*/

difference(){
    color("green") cube([10,10,10]);
    color("red") translate([3,3,3]) cube([10,10,10]);
}
color([255,0,0,0.5]) translate([4,2,3]) cube([10,10,10]);
