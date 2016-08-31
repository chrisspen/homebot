module make_mic(tol=0, condenser_length=0){
    
    cube([12.5+tol, 30.2+tol, 2.3+tol], center=true);
    
    translate([0, 30.2/2+5.3/2 + condenser_length/2, 1])
    rotate([90,0,0])
    cylinder(d=7+tol, h=5.5+tol+condenser_length, center=true, $fn=100);
    
    //color("blue")
    hull()
    for(i=[-1:2:1])
    translate([3.75-.75, 11/2*i - 7 + 3.6, 4/2 + 2.3/2])
    cylinder(d=5, h=4, center=true, $fn=100);
    
}

make_mic(tol=0, condenser_length=10);
