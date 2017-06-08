//http://www.ttelectronics.com/sites/default/files/download-files/OPB930-940-L-W_42.pdf
module make_photointerruptor_OPB940(bb=0){
    
    padding = (bb)?.5:0;
    
    difference(){
    
        union(){
            // main body
            translate([0,0,10.54/2])
            cube([12.95+padding, 6.35+padding, 10.54+padding], center=true);
            
            if(bb)
            translate([0,0,10.54/2-10])
            cube([12.95+padding, 6.35+padding, 10.54], center=true);
        }
        
        // slot
        if(!bb)
        color("red")
        translate([0,0,20/2+2.54])
        cube([3.18, 20, 20], center=true);
    }
    
    translate([2.03/2+12.95/2-2.03,0,6.35/2+2.79])
    difference(){
    
        // mount tab body
        color("blue")
        rotate([0,90,0])
        translate([0,0,0])
        cube([6.35+padding, 23.75+padding, 2.03+padding], center=true);
        
        if(!bb)
        color("green")
        for(i=[-1:2:1]) 
        //translate([0,(14.22/2+(5.84/2-3.17/2)*j)*i,0])
        translate([0,14.22/2*i,0])
        hull(){
            for(j=[-1:2:1])
            translate([0,(5.84/2-3.17/2)*j,0])
            rotate([0,90,0])
            cylinder(d=3.17, h=30, center=true, $fn=50);
        }
    }
}

make_photointerruptor_OPB940();
