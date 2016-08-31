include <../settings.scad>;
use <../openscad-extra/countersink.scad>;
use <../openscad-extra/torus.scad>;
use <../openscad-extra/screw.scad>;
use <../openscad-extra/fillet.scad>;
use <mic_mount.scad>;

module make_head_shell_bb($fn=100, contraction=0, neck=1){
    
    extra_height = 10;
    
    // main body
    translate([0,0,0])
    sphere(d=max_footprint-contraction*2, center=true);
    
    if(neck)
    translate([0,0,-(max_footprint/2+2.5+contraction+extra_height)/2])
    cylinder(d=max_footprint-contraction*2, h=max_footprint/2+2.5+extra_height+contraction, center=true);
}

module make_head_shell(thickness=2, neck=1){
    difference(){
        make_head_shell_bb(neck=neck);
        
        translate([0,0,0])
        make_head_shell_bb(contraction=thickness, neck=neck);
    }
    /*
    color("red")
    rotate([0,90,0])
    cylinder(d=3, h=300, center=true);
    */
}

module make_head_shell_side(){
    ch = 600;
    
    difference(){
    
        union(){
            difference(){
                make_head_shell();

                // middle cutout
                translate([-ch/2,0,0])
                cube([head_middle_cutout-10+ch, max_footprint+1, max_footprint*2+1], center=true);
                
                translate([0,0,-600/2 - (max_footprint/2+2.5+10) + 0.5])
                cube([600, 600, 600], center=true);
            }//end diff
            
            for(i=[0:1])
            mirror([0,i,0])
            translate([39.5, 62.5, -87+20])
            rotate([0,-90,0])
            rotate([0,0,-90])
            linear_extrude(2)
            polygon(points=[[0,0], [25, 13], [0, 67]]);        
                    
            intersection(){
                
                make_head_shell_bb();
                
                union(){
                    
                    // top mount strut
                    difference(){
                        color("blue")
                        translate([42.5,7.5*5/2,45])
                        rotate([90,0,0])
                        linear_extrude(7.5*5)
                        polygon([[0,-5/2],[0,5/2],[20*2,30*2/2],[20*2,-30*2/2]]);
                    
                        translate([42.5,0,40])
                        scale([3,1.15,1])
                        cylinder(d=20, h=50, center=true);
                    }
                
                    // side mount struts
                    for(i=[-1:2:1])
                    color("blue")
                    translate([0,45*i+1/2,0])
                    rotate([90,0,0])
                    translate([42.5,0,-2.5-1/2])
                    linear_extrude(7)
                    polygon([[0,-35/2],[0,35/2],[60,45],[60,-45]]);
                    
                    // bottom mount strut
                    color("green")
                    translate([40+2.5, 2.5-15+1, -50-5])
                    rotate([90,0,0])
                    linear_extrude(7)
                    polygon([[0,-5/2],[0,5/2],[60,30],[60,-30]]);
                    
                }
            }
        }// end diff
        
        // top strut holes
        translate([49-10,0,0])
        scale([3,1,2.5])
        rotate([90,0,0])
        cylinder(d=10, h=96.5+1, center=true, $fn=100);
        
        // top holes
        color("purple")
        for(i=[-1:2:1])
        translate([52.5-4, 15*i, 45])
        rotate([0,90,0])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=10, outer=50);
        
        // bottom hole
        color("red")
        translate([67.5-4, -15, -55])
        rotate([0,90,0])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=25, outer=50);
        
        // middle holes
        color("orange")
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([56.5-4, 45*i, 15*j])
        rotate([0,90,0])
        make_countersink(d1=2.5+0.5, d2=5+0.5, inner=14, outer=50);
    
        // middle-lower section mount holes
        for(i=[0:1])
        mirror([0,i,0])
        make_head_shell_front_holes();
        
    }// end diff
}

module make_head_shell_top_bb(){
    translate([0,0,100/2 + 7.5*5])
    cube([85-10-0.5, 300, 100], center=true);
}

module make_head_shell_top(tabs=1){
    
    offset_a = 80;
    
    difference(){
        union(){
            intersection(){
                make_head_shell();
                make_head_shell_top_bb();
            }
            
            // mount legs
            intersection(){
                sphere(d=150, $fn=100);
                color("blue")
                for(i=[-1:2:1]) for(j=[-1:2:1])
                translate([30*i,5*7*j,50+2.5])
                cube([6,6,30], center=true);
            }
            
            // cross supports
            intersection(){
                color("green")
                for(i=[-1:2:1])
                translate([(30+5)*i, 0, 40-1.25])
                cube([5-0.5,150,5/2], center=true);
                
                sphere(d=150, $fn=100);
            }

            difference(){
            
                // angled fillet for support
                intersection(){
                    color("orange")
                    for(i=[-1:2:1])
                    translate([0,0,150/2-12])
                    rotate([27*i,0,0])
                    translate([0,0,100/2])
                    cube([75-0.5, 200, 100], center=true);
                    
                    sphere(d=150, $fn=100);
                    
                    make_head_shell_top_bb();
                }
            
                // front cutout
                color("green")
                rotate([0,0,180*1])
                translate([0,0,150/2-7])
                rotate([-28+10,0,0])
                translate([0, offset_a/2, -100/2])
                cube([75-0.5-30+9.5, offset_a, 100], center=true);

                // back cutout
                color("green")
                rotate([0,0,180*0])
                translate([0,0,150/2-7])
                rotate([-28+10,0,0])
                translate([0,200/2,-100/2])
                cube([75-0.5-30+9.5, 200, 100], center=true);
            
            }//end diff


        }
        
        color("red")
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([30*i, 5*7*j, 55+3-5])
        make_countersink(inner=20, outer=20, d1=2.5+0.5, d2=5+0.5);
 
        // microphone holes
        translate([0, 0, 35])
        mic_layout();
        
        // mic mount cutout
        color("red")
        translate([0,0,35])
        make_mic_mount(tol=0.5, bb=1);
        
        // expansion mount holes
        color("blue")
        for(i=[-1:2:1])
        translate([0, 35*i, 54+3])
        cylinder(d=2, h=5.5, center=true);
        
    }//end diff
    
    if(tabs)
    color("purple")
    for(i=[-1:2:1]) for(j=[-1:2:1])
    translate([(-37.5)*i, (55-2.5)*j, 40-2.5])
    cylinder(d=15+5/2, h=0.2);

}

module make_head_shell_fb_base(){
    extra_z = 45;
    
    difference(){
        intersection(){
            make_head_shell(neck=0);
        
            color("blue")
            translate([0,-300/2,-extra_z/2])
            cube([85-10-0.5, 300, 15*5+extra_z], center=true);
        }
/*
        rotate([-90,0,0])
        rotate([0,90,0])
        torus(r1=20, r2=150/2, angle=60, endstops=1);
*/
    }
/*    
    intersection(){
        sphere(d=150);
        
        color("blue")
        translate([0,-300/2,-extra_z/2])
        cube([85-10-0.5, 300, 15*5+extra_z], center=true);
    
        rotate([-90,0,0])
        rotate([0,90,0]){
            difference(){
                torus(r1=20+2, r2=150/2, angle=60, endstops=1);
                torus(r1=20, r2=150/2, angle=60, endstops=1);
            }
        }
    }
*/
}

module make_head_shell_front(){
    
    camera_angle = 70;
    laser_angle = 95;
    led_offset = 22.6;
    
    difference(){
        union(){
            difference(){
                intersection(){
                    make_head_shell_fb_base();
                    color("blue")
                    translate([0,0,100/2-37.5])
                    cube([85, 150, 100], center=true);
                }

                // mount strut cutout for lower shell
                intersection(){
                    color("green")
                    for(i=[-1:2:1])
                    translate([30*i,-52.5,-32.5-5])
                    cube([10+0.5,5+0.5,20], center=true);
                    
                    sphere(d=150, $fn=100);
                }
            }

            // mount struts top
            mirror([0,0,1]){
            intersection(){
                color("green")
                for(i=[-1:2:1]){
                    translate([30*i, -52.5 - 6 -4/2+0.0+5, -27.5-10/2-2])
                    cube([10,10,6], center=true);
                    translate([30*i,-50.5,-14])
                    rotate([0,-90,90])
                    make_2d_corner_fillet(width=35, height=35, depth=10);
                }
                
                sphere(d=150, $fn=100);
            }
            }
            
            // mount struts bottom
            intersection(){
                color("green")
                for(i=[-1:2:1])
                translate([30*i, -52.5 - 6 -4/2+0.0, -27.5-10/2-2])
                cube([10,10,6], center=true);
                
                sphere(d=150, $fn=100);
            }
        }
        
        make_head_shell_front_middle_holes();
        
        mirror([0,0,1])
        make_head_shell_front_middle_holes();
            
        // camera cutout
        translate([0,-70,10])
        make_camera_cone(hollow=0);
        
        // laser cutout
        translate([0,-70-7,-12.5])
        make_laser_cone(hollow=0);

        // led cutout
        translate([0,-70,0])
        for(j=[-1:2:1])
        translate([led_offset*j,0,0])
        make_led_cone(hollow=0);
        
    }
    
    difference(){
        intersection(){
            union(){
                // camera cutout
                translate([0,-70,10])
                make_camera_cone(hollow=1);
                
                // laser cutout
                translate([0,-70-7,-12.5])
                make_laser_cone(hollow=1);

                // led cutout
                translate([0,-70,0])
                for(j=[-1:2:1])
                translate([led_offset*j,0,0])
                make_led_cone(hollow=1);
            }
            
            sphere(d=150, $fn=100);
        }
    
        // camera cutout
        translate([0,-70,10])
        make_camera_cone(hollow=0);
        
        // laser cutout
        translate([0,-70-7,-12.5])
        make_laser_cone(hollow=0);

        // led cutout
        translate([0,-70,0])
        for(j=[-1:2:1])
        translate([led_offset*j,0,0])
        make_led_cone(hollow=0);
    }
    
}

module make_led_cone(angle=45, hollow=0, r1=4){
    if(hollow){
        difference(){
            color("green")
            hull()
            for(i=[-1:2:1])
            translate([0,0,10*i])
            rotate([90,0,0])
            cylinder(r1=r1+1, r2=10+1, h=15-1, center=true);
            
            hull()
            for(i=[-1:2:1])
            translate([0,0,10*i])
            rotate([90,0,0])
            cylinder(r1=r1, r2=10, h=15, center=true);
        }
    }else{
        hull()
        for(i=[-1:2:1])
        translate([0,0,10*i])
        rotate([90,0,0])
        cylinder(r1=r1, r2=10, h=15, center=true);
    }
}

module make_camera_cone(camera_angle=70, hollow=0, r1=8/2, h=14, thickness=2){
    //tan(camera_angle) = (r2-r1)/h
    //r2 = tan(camera_angle)*h + r1
    if(hollow){

        difference(){
            color("green")
            rotate([90,0,0])
            cylinder(r1=r1+thickness, r2=tan(camera_angle/2)*h + r1+thickness, h=h-.1, center=true);
        
            rotate([90,0,0])
            cylinder(r1=r1, r2=tan(camera_angle/2)*h + r1, h=h, center=true);
        }
        
    }else{
        rotate([90,0,0])
        cylinder(r1=r1, r2=tan(camera_angle/2)*h + r1, h=h, center=true);
    }
}

module make_laser_cone(laser_angle=95, hollow=0, r1=13/2, h=15, thickness=2){
    if(hollow){

        difference(){
            color("green")
            rotate([90,0,0])
            cylinder(r1=r1+thickness, r2=tan(laser_angle/2)*h + r1+thickness, h=h-.1, center=true);
        
            rotate([90,0,0])
            cylinder(r1=r1, r2=tan(laser_angle/2)*h + r1, h=h, center=true);
        }
        
    }else{
        rotate([90,0,0])
        cylinder(r1=r1, r2=tan(laser_angle/2)*h + r1, h=h, center=true);
    }
}

module make_head_shell_front_middle(){
    difference(){
        union(){
            difference(){
                union(){
                    intersection(){
                        make_head_shell_fb_base();
                        color("blue")
                        translate([0,0,-100/2-40+2.5])
                        cube([85, 150, 100], center=true);
                    }

                    // mount strut
                    intersection(){
                        color("green")
                        for(i=[-1:2:1])
                        translate([30*i, -52.5, -32.5-5-5])
                        cube([10,5,20], center=true);
                        
                        sphere(d=150, $fn=100);
                    }
                }
            
                translate([0,3+.5,0])
                make_head_shell_front_middle_holes();
            
            }//end diff
            
            /*
            difference(){
                color("red")
                translate([0,0,-72])
                cylinder(d=38, h=1, center=true);
                
                translate([0,100/2,-100/2])
                cube([100, 100, 100], center=true);
            }//end diff
            */
            
            // male join tab
            translate([24,-5,-69])
            rotate([0,90-22,0])
            rotate([0,0,90])
            linear_extrude(6)
            polygon(points=[[0,0], [5, 0], [10, 2.5], [10, 5], [0, 5]]);
            
            // female join tab
            mirror([1,0,0])
            translate([24,-5,-69])
            rotate([0,90-22,0])
            rotate([0,0,90])
            linear_extrude(6)
            polygon(points=[[0,0], [5, 0], [0, 2.5-0.5]]);
                
        }
        
        // mount holes
        color("red")
        for(i=[0:1])
        rotate([0,0,180*i])
        rotate([0,21.5+180,0])
        translate([0,-5/2,73.75])
        make_countersink(d2=5+0.5);
    
    }// end diff
    
}

module make_head_shell_front_middle_holes(){
    color("red")
    for(i=[-1:2:1])
    translate([30*i,-70+13,-30-5])
    rotate([90,0,0])
    make_countersink(inner=30, outer=10, d1=2.5+0.5, d2=5+0.5);
}

module make_head_shell_back(){

    difference(){
        union(){
            difference(){
                intersection(){
                    make_head_shell_fb_base();
                    color("blue")
                    translate([0,0,100/2-37.5])
                    cube([85, 150, 100], center=true);
                }

                // mount strut cutout for lower shell
                intersection(){
                    color("green")
                    for(i=[-1:2:1])
                    translate([30*i,-52.5,-32.5-5])
                    cube([10+0.5,5+0.5,20], center=true);
                    
                    sphere(d=150, $fn=100);
                }
            }

            // mount struts top
            mirror([0,0,1])
            intersection(){
                color("green")
                for(i=[-1:2:1]){
                    translate([30*i, -52.5 - 6 -4/2+0.0+5, -27.5-10/2-2])
                    cube([10,10,6], center=true);
                    
                    translate([30*i,-50.5,18.5+25])
                    scale([1,1,3])
                    rotate([0,-90,90])
                    make_2d_corner_fillet(width=50, height=50, depth=10);
                }
                
                sphere(d=150, $fn=100);
            }
            mirror([0,0,1])
            intersection(){
                color("green"){
                    translate([0, -52.5 - 6 -4/2+0.0+5 - 10/2, -27.5-10/2-2])
                    cube([10,20,6], center=true);
                    
                    translate([0, -50.5, 18.5+25])
                    scale([1,1,3])
                    rotate([0,-90,90])
                    make_2d_corner_fillet(width=50, height=50, depth=10);
                }
                
                sphere(d=150, $fn=100);
            }
            
            // mount struts bottom
            intersection(){
                color("green")
                for(i=[-1:2:1])
                translate([30*i, -52.5 - 6 -4/2+0.0, -27.5-10/2-2])
                cube([10,10,6], center=true);
                
                sphere(d=150, $fn=100);
            }
            intersection(){
                color("red")
                translate([0, -52.5 - 6 -4/2+0.0, -27.5-10/2-2])
                cube([10,20,6], center=true);
                
                sphere(d=150, $fn=100);
            }
        }
        
        
        //make_head_shell_front_middle_holes();
        
        //mirror([0,0,1])
        //make_head_shell_front_middle_holes();
        
        // middle mount holes
        color("red")
        for(i=[-1:2:1])
        translate([0, -64.5+4, 35*i])
        rotate([90,0,0])
        make_countersink(inner=14, d1=2.5+0.5, d2=5+0.5);
            
    }
    
    
}

module make_head_shell_front_lower_base($fn=100, thickness=2, tabs=1){
    
    slope_length = 15;
    m = 7.5;
    m2 = 37.5;
    
    intersection(){
        make_head_shell(neck=1);
    
        color("blue")
        translate([0,110,-115+2.5+45])
        cube([85-10, 300, 20], center=true);
    }
    
    intersection(){
        difference(){
            translate([0,0, -57.5]){
                /*
                color("orange")
                translate([-50,0,slope_length/2])
                cylinder(
                    r1=150/2,//bottom
                    r2=(150-slope_length*3)/2,//top
                    h=slope_length,
                    center=true);*/
                
                difference(){
                    color("purple")
                    translate([0,0,slope_length/2])
                    cylinder(
                        r2=(150-slope_length*3)/2-m,//top
                        r1=150/2+m,//bottom
                        h=slope_length+10,
                        center=true);
                    
                    color("green")
                    translate([0,0,slope_length/2-1])
                    cylinder(
                        r1=150/2-thickness+m,//bottom
                        r2=(150-slope_length*3)/2-thickness-m,//top
                        h=slope_length+0.1+10,
                        center=true);
                    
                }//end diff
            }
            translate([0,0,10]) sphere(r=150/2 + 2);
        }//end diff
      
        intersection(){
            color("blue")
            translate([0,120,-20])
            cube([85-10, 300, 15*6], center=true);
                
            cylinder(d=150, h=500, center=true, $fn=100);
        }
        
    }
    
    // mount tabs
    translate([0,0,10]){
        
        difference(){
            intersection(){
                union(){
                    color("blue")
                    for(i=[0:1])
                    mirror([i,0,0])
                    rotate([0,0,30-3])
                    translate([0,60,-60-7.5])
                    rotate([-90,0,0])
                    linear_extrude(25)
                    polygon(points=[[0,0],[0,20],[-5,20],[-15,10+2.5],[-15,10-2.5],[-5,0],[-5,-15],[0,-15]]);
                }
            
                translate([0,0, -57.5-10])
                color("orange")
                translate([0,0,slope_length/2])
                cylinder(
                    r2=(150-slope_length*3)/2-m2,//top
                    r1=150/2+m2,//bottom
                    h=slope_length+50,
                    center=true);
                
                color("lightgreen")
                translate([0,0,-75-2.5])
                difference(){
                    cylinder(d=150-2*2, h=20+15, center=true);
                    cylinder(d=150-2*2-5*2, h=100, center=true);
                }
            }
            
            make_head_shell_front_holes();
        }// end diff
    }
    
    // corner holders
    if(tabs)
    for(i=[-1:2:1])
    rotate([0,0,37*i])
    translate([0, 71.5, -77.5+0.1])
    cylinder(d=20, h=0.2, center=true);
    
    // extra bottom rim
    translate([0,0,-77])
    intersection(){
        difference(){
            cylinder(d=150, h=1, center=true, $fn=100);
            cylinder(d=150-2*2-5*2, h=1.1, center=true, $fn=100);
        }
        color("blue")
        translate([0,120,-40])
        cube([85-17, 300, 15*6], center=true);
    }
        
}

module make_head_shell_front_lower($fn=100, thickness=2, tabs=1){
    
    m = 60/-90;//y/x
    y1 = 3.5;
    z1 = m*y1;
    
    difference(){
        union(){
            make_head_shell_front_lower_base($fn=$fn, thickness=thickness, tabs=tabs);
        
            difference(){
                // speaker mount bulk
                intersection(){
                    color("blue")
                    translate([0,60+y1,-50+z1])
                    rotate([-34,0,0])
                    hull()
                    for(i=[0:1])
                    translate([0,15*i,-2.5])
                    cylinder(d=18.5+3, h=4+3, center=true);
                
                    cylinder(d=150, h=200, center=true, $fn=100);
                    
                }//end diff
                
                // speaker cutout
                color("purple")
                translate([0,60+y1,-50+z1])
                rotate([-34,0,0])
                translate([0,0,-2.5])
                cylinder(d=18.5, h=4, center=true);
                
                // speaker slot cutout
                color("purple")
                translate([0,60+y1,-50+z1])
                rotate([-34,0,0])
                hull()
                for(i=[0:1])
                translate([0,-10*i,-2.5])
                cylinder(d=18, h=4, center=true);
            
                // speaker back cutout
                color("purple")
                translate([0,60+y1,-50+z1])
                rotate([-34,0,0])
                hull()
                for(i=[-1:1:1])
                translate([0,-7*i,-5.5])
                cylinder(d=15, h=10, center=true);
                
            }// end diff
            
            // board mount bulk
            color("orange")
            rotate([0,0,29])
            translate([0,60+y1,-50+z1-1])
            rotate([-33,0,0])
            translate([0,-1.7,0]){
                difference(){
                    cube([5,21.5,5], center=true);
                    
                    // mount holes            
                    for(i=[-1:2:1])
                    translate([0, (8.5+2.2/2)*i/2, -5/2])
                    cylinder(d=2, h=5, center=true);
                    
                    
                    // through holes            
                    for(i=[-1:2:1])
                    translate([-0.8, (5-2.1/2)*i/2, -5/2])
                    cylinder(d=2.1, h=5, center=true);
                }
            }
        }
        
        // speaker holes
        rotate([-32,0,0])
        translate([0,79,0])
        union(){
            cylinder(d=2, h=50, center=true);
            for(i=[0:7]){
                rotate([0, 0, 360/8*i])
                translate([3.5, 0, 0])
                cylinder(d=2, h=50, center=true);
            }
        }
    
    }//end diff
    
    /*
    // mock board
    rotate([0,0,29])
    translate([0,60+y1,-50+z1-1])
    rotate([-33,0,0])
    translate([-10,-2,-3.5])
    cube([25,15,2],center=true);
    */
    
}

module make_head_shell_back_lower($fn=100, thickness=2, tabs=0){
    make_head_shell_front_lower_base($fn=$fn, thickness=thickness, tabs=tabs);
}

module make_head_shell_front_holes(){
    
    color("red")
    for(i=[-1:2:1])
    rotate([0,0,35*i])
    translate([0,0,-75-2.5])
    rotate([-90,0,0])
    translate([0,0,74.9])
    //cylinder(d=2.5, h=200, center=true);
    make_countersink(d2=5+0.5);
    
}

module make_head_shell_neck_mesh($fn=50){
    /*
    //intersection(){
        make_head_shell(neck=0);
    
        cylinder(d=130, h=120, center=true);
    
        color("blue")
        translate([0, -300/2, -15*5])
        cube([85-0.5, 300, 15*5], center=true);
    //}
    */
    /*
    translate([0,0,-60]){
        for(i=[0:100])
        translate([
            0,
            -sin(i/100*90.)*25 + 30 + sin(i/1*25)*5,
            i/5])
        torus(r2=70);
    }*/

    intersection(){

        translate([0,0,-60])
        rotate_extrude()
        translate([-35-30,0,0])
        polygon(points=concat(
            [for(i=[0:100]) [-sin(i/100*90.)*25 + 30 + sin(i/1*25)*5, i/5]],
            [for(i=[100:-1:0]) [-sin(i/100*90.)*25 + 30 + sin(i/1*25)*5 + 2, i/5]]
        ));
        
        color("blue")
        translate([0, -300/2-20, -15*5])
        cube([85-0.5, 300, 15*5], center=true);
    }
}

//translate([150,0,0]) make_head_shell_bb();

//make_head_shell_side();

//make_head_shell_top();

//translate([0,0,0]) make_head_shell_front();
/*
translate([0,0,-70]){

    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-47.5,0])
    //translate([0,-27.5,0])
    rotate([0,-90,-90])
    import("../printable/head_face_laser_mount_20160214.stl");

    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([22.5,-61.25,0])
    //translate([0,-27.5,0])
    rotate([0,-90,-90])
    import("../printable/head_face_led_mount_20160214.stl");
    
    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-56.75,10])
    rotate([90,0,0])
    color("purple")
    make_rpi_camera();
    
    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-56.75,10])
    rotate([90,0,0])
    color("purple")
    make_rpi_camera();
    
    // rpi axis marker
    color("purple")
    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-56.75,10])
    rotate([90,0,0])
    cylinder(d=5, h=200, center=true);
    
    color("blue")
    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-69,-12.5])
    rotate([-90,0,0])
    make_laser_03015L();
    
    // laser axis marker
    color("blue")
    translate([0,0,70])
    rotate([head_vertical_angle_tilt,0,0])
    translate([0,-69,-12.5])
    rotate([90,0,0])
    cylinder(d=12, h=200, center=true);

}
*/

//make_head_shell_fb_base();

//make_head_shell_front_middle();

make_head_shell_front_lower();

//make_head_shell_back_lower();

//make_head_shell_back();

//make_head_shell_neck_mesh();

//make_head_shell_side();

//make_head_shell_top();

//translate([0,0,-10+0.5]) make_head_shell_front_lower();

/*
translate([40,0,-65])
rotate([0,90,180])
import("../printable/neck_strut_servo_20150925.stl");
*/
