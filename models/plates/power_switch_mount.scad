include <../settings.scad>;
use <../electronics/pololu_pushbutton.scad>;
use <countersink.scad>;

module make_power_switch_holster(){
    color("red"){
        difference(){
            union(){
                cube([u*4, u*2-tolerance, u*1.5-tolerance+1], center=true);
                
                difference(){
                    // restraining bulk
                    translate([0, 0, -u*1.4])
                    cube([u*4, u*3.75, u*1+1], center=true);
            
                    translate([
                        5,
                        0,//u
                        0]){
                            
                        translate([0, 0, -u*1.25+1.9])
                        //translate([-u*2, 0, u*1.3])
                        rotate([180,0,90])
                        scale([1.05,1.5,1.1])
                        make_pololu_2808_pushbutton();
                                
                        translate([0, 0, -u*1.25+1.9])
                        //translate([-u*2, 0, u*1.3])
                        rotate([180,0,90])
                        scale([.95,1.5,.95])
                        make_pololu_2808_pushbutton();
                        
                    }
                }//end diff
            }
            
            
            translate([u*1.5, u*.5, -u*.7])
            rotate([0,180,0])
            make_countersink(inner=u*5, outer=u);
            
            translate([u*1.5, -u*.5, -u*.7])
            rotate([0,180,0])
            make_countersink(inner=u*5, outer=u);
            
        }//end diff
    }
    
    
    
}

module make_power_switch_mount(show_switch=0){
    width = u*16;

    difference(){
    
        // main body
        union(){
            cube([u, width, u], center=true);
            translate([u*.50,0,0])
            cube([u*2, u*14, u], center=true);
            
            for(i=[-1:2:1])
            translate([0, (width*.5 - u*.5)*i, u])
            cube([u, u, u*3], center=true);
        }
        
        // frame mount holes
        color("red"){
            for(i=[-1:2:1])
            translate([-u*.3, u*7.5*i, 0+u*1.5])
            rotate([0,-90,0])
            make_countersink(inner=u*3, outer=u*3);
        }
        
        // generic mount holes
        color("blue"){
            for(i=[0:13])
            translate([-u*1, u*i - u*6.5, 0])
            rotate([0,-90,0])
            make_countersink(inner=u*3, outer=u*0);
        }
        
    }
    
    if(show_switch){
        translate([-u*2, 0, u*1.3])
        rotate([90,0,-90])
        make_pololu_2808_pushbutton();
        
        translate([-u*1.25,0,u*1.5])
        rotate([0,90,0])
        make_power_switch_holster();

    }
}

//color("blue")
translate([0,0,0])
rotate([0,-90,0])
make_power_switch_mount(show_switch=0);

//translate([-u*1.5, 0, -u*1.25])make_power_switch_holster();
