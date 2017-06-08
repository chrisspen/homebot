include <../settings.scad>;
use <../openscad-extra/src/countersink.scad>;
use <../electronics/photointerrupter.scad>;

module neck_sensor_mount(){

    lower_h = 5+.25+2.5;
    upper_h = 5.5+2.5;
    edge_flange = 5+2.5;

    if(1)
    difference(){
        color("green")
        intersection(){
            difference(){
                // main mass
                cube([20,35+edge_flange*2,30], center=true);
                
                translate([2.5+5,0,0])
                cube([5+.5,35+.5,50], center=true);
            }//end diff

            union(){
                // lower indent
                translate([-(-50+2.5),0,-5])
                cylinder(d=100-6, h=lower_h, center=true, $fn=100);
                
                // upper main mass
                translate([-(-50+2.5),0,upper_h/2-7.4+6])
                cylinder(d=100, h=upper_h, center=true, $fn=100);
            }

        }

        // mount holes
        color("red")
        for(i=[-1:2:1])
        translate([3,15*i,-5])
        rotate([0,-90,0])
        make_countersink(inner=10);
        
        // outward cutofff
        color("red")
        translate([-10/2+1,0,0])
        cube([10,100,50], center=true);

        // wire holes
        color("red")
        for(j=[-1:2:1])
        translate([7.6, 35/2*j, 0])
        hull()
        for(i=[-1:2:1])
        translate([0,2*i,0])
        cylinder(d=3, h=30, center=true);

        color("purple")
        translate([-1.5,0,3.5])
        rotate([180,0,0])
        make_photointerruptor_OPB940(bb=1);

    }// end diff

}

rotate([0,-90,0])
neck_sensor_mount();
