//Pololu drv8838 single brushed DC motor driver carrier

module make_pololu_drv8838(tol=0, header_length=0, headers=1){

    //smd bb
    translate([0,0,1])
    cube([13, 5+6, 1+tol], center=true);
    
    //board
    cube([13, 10.3, 1+tol], center=true);
    
    if(headers)
    //headers
    color("blue")
    for(i=[0:1])
    mirror([0,i,0])
    translate([0, 6.5 + header_length/2 + 0.35, 0.75])
    cube([13+0.5, 3.5+header_length, 2.5+tol], center=true);
    
    //through pin bb
    color("orange")
    for(i=[-1:2:1])
    translate([0, 3.9*i, -2])
    cube([13,2.5,3], center=true);
}

make_pololu_drv8838(tol=0.5, header_length=0);
