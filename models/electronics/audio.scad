
use <../plates/countersink.scad>;

module speaker(buffer=0){
    cylinder(d=19+buffer, h=4+buffer, center=true, $fn=100);
    translate([0,0,3])
    cylinder(d=15+buffer, h=3+buffer, center=true, $fn=100);
    translate([0,0,-3])cylinder(d=17.5, h=3, center=true);
}

module make_adafruit_mono(speaker_pos=1, buffer=0){

    speaker_offset_y = 3;
    
    difference(){
        union(){
            // main body
            cube([16+buffer, 25+buffer, 2+buffer], center=true);
            translate([0,0,-10/2+2.5])
            cube([14+buffer, 25+buffer, 10+buffer], center=true);
            
            // speaker terminal
            color("blue")
            translate([0,-8.5-10/2,9/2+2/2])
            cube([8+buffer, 8+buffer+10, 9+buffer], center=true);
        }
        
        /*/ holes
        color("blue")
        for(i=[-1:2:1])
        translate([10.5/2*i,-10,0])
        cylinder(d=2.25, h=10, center=true, $fn=100);
        */
    }
    
    // plug housing
    translate([0+5/2,25/2+16/2-3 + 5/2,2])
    cube([16+5+buffer,16+buffer+5,2+buffer], center=true);
    
    if(speaker_pos==1)
    translate([0,speaker_offset_y,-7]){
        speaker(buffer=buffer);
    }
    
    if(speaker_pos==2)
    translate([0,6+speaker_offset_y,9]){
        rotate([0,180,0])
        speaker(buffer=buffer);
    }
}

module make_speaker_mount_bridge(){
    
    bridge_width = 30;
    
    difference(){
        color("red")
        translate([-10-.5, 3, -2+1.5])
        rotate([0,40,0]){
        difference(){
            translate([-5,0,5/2])
            cube([10, bridge_width, 10+5], center=true);
            translate([-0,0,-5])
            cube([10, bridge_width+1, 10], center=true);
            
            translate([-19+10, 0, -2.5])
            rotate([0,-90,0])
            make_countersink();
            translate([-19+10, 10, -2.5])
            rotate([0,-90,0])
            make_countersink();
        }
        }
        
        make_speaker_mount(bb=1);
    
        make_speaker_mount_holes();
    }
}

module make_speaker_mount_holes(){

    color("blue")
    translate([9.0,15,7.5])
    rotate([0,90,0])
    make_countersink();
    
    color("blue")
    translate([9.0,-7.5,5.5])
    rotate([0,90,0])
    make_countersink();
    
}

module make_speaker_mount(side=0, bb=0){
    
    difference(){
        
        union(){
            translate([0,0,5])
            cube([20,40,14], center=true);
            
        }
        
        if(!bb){
                
            translate([0,-6,0])
            make_adafruit_mono(speaker_pos=2, buffer=0.5);
            
            if(side==1)
            translate([0,30/2,0])
            cube([30,30,30], center=true);
            
            if(side==2)
            translate([0,-30/2,0])
            cube([30,30,30], center=true);
            
            if(side==3)
            translate([30/2,0,0])
            cube([30,60,30], center=true);
            
            if(side==4)
            translate([-30/2,0,0])
            cube([30,60,30], center=true);
            
            // horz wire routing
            translate([0,-8.5,3.5])
            cube([15, 25, 2.5], center=true);
        
            // vert wire routing
            color("blue")
            translate([0,3,8])
            cube([18, 5, 10], center=true);
            
            make_speaker_mount_holes();
            
        }
    }

}

/*
color("green")
translate([0,-9,0])
make_adafruit_mono(speaker_pos=2);
*/

translate([0,-3,0]){
//make_speaker_mount(side=3);
//make_speaker_mount(side=4);
}


make_speaker_mount_bridge();

