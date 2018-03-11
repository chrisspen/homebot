include <../settings.scad>;
use <tray.scad>;
use <../electronics/battery.scad>;
use <../openscad-extra/src/countersink.scad>;

module make_battery_tray2_bb(extra_height=0, extra_width=0, extra_length=0, extra_d=0){
    extra_length0 = 10;
    intersection(){
        translate([0, extra_length0/2+15, 0])
        cube([70-tolerance+extra_width, 100+extra_length0+extra_length, 30-tolerance+extra_height], center=true);
        
        cylinder(d=150+extra_d, h=30*2-tolerance+extra_height, center=true, $fn=100);
    }
}

module make_battery_tray2_bb_inverted(extra_height=0, extra_width=0, extra_length=0, extra_d=0){
    extra_length0 = 10;
    difference(){
    
        cylinder(d=150+extra_d, h=30+extra_height, center=true, $fn=100);
        
        translate([0, extra_length0/2+15, 0])
        cube([70+extra_width, 100+extra_length0+extra_length, 30*2+extra_height], center=true);
        
        translate([0, -(100*2+extra_length)/2+40, 0])
        cube([100*2+extra_width, 100*2+extra_length, 30*2+extra_height], center=true);
        
    }
}

module make_battery_tray_receiver(negative=0, bb=0){
    difference(){
        intersection(){
            
            make_battery_tray2_bb(extra_height=10*negative, extra_width=5*negative, extra_length=5*negative);
            
            union(){
                // backplane
                translate([0,-30-5/2-tolerance/2+tolerance*negative,0])
                cube([80+negative*10+50, 5+tolerance+1*negative, 30+tolerance+10*negative], center=true);
                
                // male receiver plug
                translate([40/2+5-tolerance*negative, -34.5+tolerance*negative, 0])
                cube([40,40,30+tolerance+10*negative], center=true);
            }
        }
        // hallow out
        if(bb==0)
        color("red")
        translate([20,-35+10,0])
        cube([20, 10, u*10], center=true);
        
        // hallow out
        if(bb==0)
        color("blue")
        translate([20, -35+5, u*3])
        cube([20, 15, u*10], center=true);
        
        // screw holes
        if(bb==0)
        for(i=[-1:2:1]){
            translate([u*6.5*i, -50, 0])
            rotate([90, 0, 0])
            make_countersink();
            translate([u*6.5*i, -50, u*2])
            rotate([90, 0, 0])
            make_countersink();
            translate([u*5*i, -50, -u*2.5])
            rotate([90, 0, 0])
            make_countersink();
        }
        
        // male connector cutout
        if(bb==0)
        color("red")
        translate([20, -17, -u*1.25])
        rotate([-90,0,0])
        make_blade_connector_male(bb=1);
        
    }// end diff
}

module make_battery_tray_cartridge(show_holes=1){
    difference(){
        union(){
            difference(){
                make_battery_tray2_bb();
                
                if(1)
                make_battery_tray_receiver(negative=1, bb=1);
            
                // cavity cutout
                //color("green")
                if(1)
                translate([0,0,0])
                difference(){
                    make_battery_tray2_bb(extra_width=-2, extra_d=-2, extra_length=-1, extra_height=-2);
                    translate([-1,1,0])
                    make_battery_tray_receiver(negative=1);
                }
                
            }//end diff
            
            // rear fillet
            if(1)
            translate([2.5,-21,0])
            cube([2,15,28], center=true);
                    
            // button container
            if(1)
            translate([9.5,73.5,0])
            rotate([0,0,-8]) _make_button_container_outer();
            
            // join screw hole bulkhead
            if(1)
            translate([4,-11.5,0])
            cube([5,5,30-tolerance], center=true);
            
            // join screw hole bulkhead, center
            //translate([4,30,0])
            //cube([5,5,30-tolerance], center=true);
            
            // join screw hole bulkhead
            if(1)
            translate([0,71.5,0])
            cube([5,5,30-tolerance], center=true);
            
            // female plug bulk container
            if(1)
            difference(){
                translate([20,-8,-6-5/2-1])
                cube([28+1,10,10], center=true);
                
                // female plug bulk container, post cutout
                color("red")
                translate([31,-1,10.25+1])
                cube([7.5,6.5,50], center=true);
            }
            
            // grab hole bulk
            if(1)
            intersection(){
                cylinder(d=150, h=30*2-tolerance, center=true, $fn=100);
                union(){
                    for(i=[-1:2:1])
                    rotate([0,0,-18*i])
                    translate([0,79,0])
                    sphere(d=20, $fn=100);
                }
            }
        }
        
        // female plug cutout
        if(1)
        color("red")
        translate([0,0,-25/2-5/2]) 
        translate([20, -10+1, 6])
        rotate([0,0,180])
        make_blade_connector_female(
            bb=1,
            extra_x=-3*0,
            extra_y=10*0,
            extra_z=5,
            extra_x2=-3,
            extra_y2=10
        );
        
        // screw holes
        if(1)
        translate([0,0,11.5+5/2])
        color("blue"){
            translate([4,-11.5,0])
            make_countersink(inner=100);
            
            // screw hole center
            //translate([4,30,0])
            //make_countersink();
            
            translate([0,71.5,0])
            make_countersink(inner=100);
        }
    
        // grab holes
        if(1)
        if(show_holes)
        for(i=[-1:2:1]){
            rotate([0,0,-18*i])
            translate([0,79,0]){
                difference(){
                    sphere(d=20-2, $fn=100);
                    rotate([0,0,-30*i])
                    translate([-20/2*i,0,0])
                    cube([20,20,20], center=true);
                }
            }
        }
        
        // button cutout
        if(1)
        translate([9.5,73.5-4.5,0])
        _make_button_container_inner();
        
        // angle cutout of fillet
        if(1)
        color("blue")
        translate([4.5,-14,0])
        rotate([0,0,-7])
        translate([5/2,-25/2,0])
        cube([5,25,50], center=true);
        
    }// end diff
    
}

module make_blade_connector_male(bb=0){
    bb_extra = 10;
    
    translate([0,(bb?bb_extra:0)/2,0])
    cube([27, 9.8+(bb?bb_extra:0), 3], center=true);
    
    color("blue")
    if(bb){
        translate([0,10/2,0])
        cube([22, 5.13+10, 12+10], center=true);
    }else{
        for(i=[0:4])
        translate([5*i-10, 0, 4.5-3])
        cube([0.65, 5.13, 12], center=true);
    }
}

module make_blade_connector_female(bb=0, extra_x=0, extra_y=0, extra_z=0, extra_x2=0, extra_y2=0, extra_z2=0){
    difference(){
        union(){
            // small center bulk
            translate([0, extra_y/2, -extra_z/2])
            cube([25.4+extra_x+0.5, 7.85+extra_y, 12+extra_z], center=true);
            
            // small center bulk, forward cutout
            color("blue")
            translate([0, extra_y2/2, -extra_z/2])
            cube([25.4+extra_x2, 7.85+extra_y2, 12+extra_z], center=true);
            
            // outer ridge bulk
            color("blue") translate([0, -.41+.85/2, .63 + extra_z2/2]) cube([27+.5, 7+.85, 10.75 +extra_z2], center=true);
        }
        
        if(!bb)
        for(i=[0:4])
        color("blue")
        translate([5*i-10, 1, -1])
        cube([0.65, 7, 12], center=true);
    }
    
    // back slot
    color("gray")
    translate([13.5-10.7-0.2, -3.6, -5])
    rotate([0,0,-90])
    linear_extrude(height=15 + extra_z2)
    polygon(points=[[0,0.45],[1.5,4.5/2],[1.5,-4.5/2],[0,-0.45]]);
    
}

module make_turnigy_1700mAh_lipo(){
    translate([0,0,0])
    cube([30, 89, 17], center=true);
}

module make_turnigy_2000mAh_lipo(){
    translate([0,0,0])
    cube([34, 88, 17], center=true);
}

module make_turnigy_nano_tech_3000mAh_2S2P(){
    translate([0,0,0])
    cube([35, 91, 26], center=true);
}

module make_ZIPPY_Flightmax_2500mAh(){
    translate([0,0,0])
    cube([31, 99, 26], center=true);
}

module make_battery_tray3_cartridge_top(show_holes=1){
    difference(){
        make_battery_tray_cartridge(show_holes=show_holes);
        translate([0,0,-100/2])
        cube([100,200,100], center=true);
    }
}

module make_battery_tray3_cartridge_bottom(show_holes=1){
    difference(){
        union(){
            make_battery_tray_cartridge(show_holes=show_holes);
                    
            // battery holder mount stubs
            color("red")
            translate([-0,31,-7])
            for(i=[-1:2:1]){
                for(j=[-1:2:1]){
                    translate([30.5*-i,32*j,-5])
                    cube([6.5,6,5], center=true);
                }
            }
        }
        translate([0,0,100/2])
        cube([100,200,100], center=true);

        for(i=[-1:2:1])
        translate([30.5*i,31,-10])
        make_3cell_lipo_holder_mount_holes();        

    }// end diff

}

module _make_button_container_outer(){
    difference(){
        // main bulk
        scale([1,1.3,1])
        sphere(r=15/2);
        // cut sphere in half
        translate([0,20/2,0]) cube([20,20,20],center=true);
    }
}

module _make_button_container_inner(){
    // cutout for button box
    color("blue") cube([6.2+0.5, 4+0.5, 6.2+0.5], center=true);
    
    // wire slots
    translate([2.8,-20/2,0]) cube([1.5, 20, 8], center=true);
    translate([-2.8,-20/2,0]) cube([1.5, 20, 8], center=true);

    // button cutout
    translate([0,-.25+2,0]) rotate([90,0,0]) cylinder(d=3.5+0.5, h=5, center=true);
    translate([0,20/2,0]) rotate([90,0,0]) cylinder(d=2, h=20, center=true);
}

module make_battery_nub(){
    difference(){
        union(){
            difference(){
                make_battery_tray2_bb_inverted();
                
                difference(){
                    cylinder(d=150-2, h=30*2, center=true, $fn=100);
                    cube([70+10, 100*2, 100*2], center=true);
                }
                
                for(i=[-1:2:1]) for(j=[-1:2:1])
                translate([(70/2 + 5/2)*i, 60+2, 10*j])
                rotate([-90,0,0])
                make_countersink(d2=screw_head_diameter+0.5);
                
            }// end diff
            
            // battery mount screw hole bulk
            intersection(){
                translate([0,55,0])
                cube([u*30, 6, 6], center=true);
                
                make_battery_tray2_bb_inverted();
            }
        }
    
        for(i=[-1:2:1])
        translate([(45-2)*i,55,0])
        rotate([0,90*i,0])
        make_countersink(d2=screw_head_diameter+0.5);
        
    }//end diff
}

module make_SBS_LI_74(){
    color("blue")
    cube([45, 90, 5], center=true);
}

module make_battery_tray3a_cartridge_top(){
    import("../printable/battery_tray2_cartridge_top_20160522.stl");
}

module make_3cell_lipo_balancer(){
    cube([22.5, 50.5, 5], center=true);
}

module make_3cell_lipo_holder(){
    
    // main bottom plate
    difference(){
        union(){
            color("purple")
            translate([0,0,1/2])
            cube([67, 60, 2], center=true);
                        
            // straight through support with screw ends
            for(i=[-1:2:1])
            color("green")
            translate([30.5*i,0,2])
            cube([6, 60+5*2, 5], center=true);
                
            // straight through support without screw ends
            for(i=[-1:2:1])
            color("green")
            translate([30.5/2*i,0,2])
            cube([6, 60, 5], center=true);
            
            // curved holding arms
            difference(){
                
                for(j=[-1:1:1])
                for(i=[-1:2:1])
                translate([30.5/2*i,21*j,11])
                rotate([0,90,0])
                color("blue")
                difference(){
                    cylinder(d=18.5+2, h=5, center=true, $fn=100);
                    cylinder(d=18.5, h=5*2, center=true, $fn=100);
                }
                    
                translate([0,0,100/2+16])
                cube([100,100,100], center=true);
            }
        }

        // cutouts for brass tabs
        color("pink")
        for(j=[0:1:3])
        for(i=[-1:2:1])
        translate([(67/2-0.5/2)*i,21*j-21,0])
        cube([0.5,5,200], center=true);
            
        // pho-battery cutouts
        color("red")
        for(i=[0:2])
        translate([0,21 - 21*i,11])
        rotate([90,0,90])
        make_battery_18650(tab=10);
    
        // mount hole cutouts
        for(i=[-1:2:1])
        color("red")
        translate([30.5*i,0,-5.35])
        make_3cell_lipo_holder_mount_holes(d1_buffer=0.5);

    }

}

module make_3cell_lipo_holder_mount_holes(d1_buffer=0){
    for(i=[-1:2:1])
    translate([0,-32*i,9.5])
    make_countersink(d1=2.5+d1_buffer, inner=100);
}

module make_3cell_lipo_holder_bottom_support(){
    
    // straight through support with screw ends
    difference(){
        color("orange")
        translate([0,0,2])
        cube([6, 60+5*2, 5.5], center=true);
        
        // mount mast cutout
        color("red")
        for(i=[-1:2:1])
        translate([0,-32*i,1.75-1.5])
        cube([6.5, 6.5, 6], center=true);

        make_3cell_lipo_holder_mount_holes(d1_buffer=0.5);
        
        // cutouts for brass tabs side
        color("pink")
        for(j=[0:1:3])
        translate([-2.75,21*j-21,0])
        cube([0.5,5,200], center=true);
        
        // cutouts for brass tabs bottom
        color("pink")
        for(j=[0:1:3])
        translate([0,0,-3.25])
        rotate([0,90,0])
        translate([-2.75,21*j-21,0])
        cube([0.5,5,200], center=true);
    }

}


/*
color("gray")
translate([0,0,-1/2])
cylinder(d=150, h=1, center=true, $fn=100);

color("purple")
translate([0,0,0])
import("../printable/horizontal_mount_20150816.stl");
*/

//make_battery_tray3_cartridge_bottom();
difference(){
import("../printable/battery_tray2_cartridge_bottom_20160522.stl");
if(1)
color("blue")
translate([0,0,0])
cube([100,60,100], center=true);
}


translate([0,31,40-30])
make_3cell_lipo_holder();

for(i=[-1:2:1])
translate([30.5*i,31,4.65])
make_3cell_lipo_holder_bottom_support();

if(1)
translate([80,0,18]) rotate([0,180,0])
make_battery_tray3a_cartridge_top();
//make_battery_tray3_cartridge_top();

if(1)
translate([-50+50,30,7])
color("green")
make_3cell_lipo_balancer();

if(0)
translate([10,18,20])
make_SBS_LI_74();

//translate([31,30,0])rotate([0,0,90])
if(0)
color("red")
for(i=[0:2])
translate([0, 52 - 21*i, 15+6])
rotate([90,0,90])
make_battery_18650(tab=1);

// bottom contrast
color("blue")
translate([0,0,-10])
cube([100,150,10], center=true);
