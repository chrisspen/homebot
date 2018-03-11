include <recharge_mount_settings.scad>;
use <../openscad-extra/src/countersink.scad>;
use <../openscad-extra/src/rounded.scad>;
use <cross_plate.scad>;
use <../openscad-extra/src/wedge.scad>;
use <../openscad-extra/src/torus.scad>;

module make_recharge_shell(){

    difference(){
        color("purple")
        translate([0, -u*13.5 + y_offset, 0])
        make_recharge_mount_bb();
        
        // interior rim cutout
        translate([0, -u*13.5 + y_offset, 0])
        cylinder(d=150-u*2, h=u*5, center=true, $fn=75);
    }
}

module make_recharge_mount_bb(width=u*30, height=u*4, thickness=0, h_extend=0, back_extend=0){
    intersection(){
        color("orange")
        cylinder(d=150, h=height+h_extend, center=true, $fn=100);
        
        color("blue")
        translate([0, u*12-back_extend, 0])
        cube([width, u*8 + back_extend*2, height*2+h_extend], center=true);
    }
}

module make_recharge_rounded_box(box_r=2, box_tol_y=1.5, box_tol_xz=0, terminal_extension=0, y_stretch=u){

    width = recharge_mount_width;
    height = recharge_mount_height;

    color("blue")
    hull(){
        translate([0, 0, 0])
        //rounded_cube([recharge_mount_width-u, u, recharge_mount_height-u], r=2, center=true, $fn=50);    
        rounded_cube([width-u-tolerance*box_tol_xz, u-tolerance*box_tol_y, height-u-tolerance*box_tol_xz], r=box_r, center=true, $fn=50);
        
        translate([0, y_stretch, 0])
        //rounded_cube([recharge_mount_width-u, u, recharge_mount_height-u], r=2, center=true, $fn=50);    
        rounded_cube([width-u-tolerance*box_tol_xz, u-tolerance*box_tol_y, height-u-tolerance*box_tol_xz], r=box_r, center=true, $fn=50);
    }


}

module make_recharge_plug_female(struct_thicken=0, show_holes=1, h_extend=0, box_tol_xz=2.5, show_detail=1, show_indent=1){
    
    width = recharge_mount_width;//u*8;
    height = recharge_mount_height;//u*4;
    y_offset = -2;
    terminal_extension = recharge_plug_terminal_extension;
    
    box_r = recharge_plug_box_r;
    //box_tol_xz = 2.5; // little too big
    box_tol_xz = recharge_plug_female_box_tol_xz;
    box_tol_y = recharge_plug_female_box_tol_y;
    
    difference(){

        union(){
        
            // outer solid rim
            difference(){
                
                color("purple")
                translate([0, -u*13.5 + y_offset, 0])
                make_recharge_mount_bb(h_extend=h_extend, back_extend=1);

                // interior rim cutout
                translate([0, -u*13.5 + y_offset, 0])
                cylinder(d=150-u*2, h=u*5, center=true, $fn=75);
                    
            }

            // reed switch terminal blocks
            if(show_detail)
            color("blue"){
                translate([u*2,-0,u*2])
                rotate([0,90,180])
                difference(){

                    linear_extrude(height=u, center=true)
                    polygon(points=[[0,0],[0,u],[u,u],[u*2,0]], paths=[[0,1,2,3,0]]);

                    translate([u*.5-.5,0.5,0])
                    cylinder(d=2, h=u*4, center=true);

                    translate([u*.5,0,0])
                    rotate([90,0,0])
                    cylinder(d=2.12, h=u*4, center=true);
                    
                }
                mirror([0,0,1])
                translate([u*2,-0,u*2])
                rotate([0,90,180])
                difference(){

                    linear_extrude(height=u, center=true)
                    polygon(points=[[0,0],[0,u],[u,u],[u*2,0]], paths=[[0,1,2,3,0]]);

                    translate([u*.5-.5,0.5,0])
                    cylinder(d=2, h=u*4, center=true);

                    translate([u*.5,0,0])
                    rotate([90,0,0])
                    cylinder(d=2.12, h=u*4, center=true);
                   
                }
            }
        }
        
        if(show_holes){
            
            // mounting holes
            translate([0, -u*13.5 + y_offset, 0])
            make_recharge_mount_holes();
            
            translate([recharge_mount_terminal_offset, u*5-recharge_mount_terminal_extend, 0])
            rotate([-90,0,0])
            make_countersink();
            
            translate([-recharge_mount_terminal_offset, u*5-recharge_mount_terminal_extend, 0])
            rotate([-90,0,0])
            make_countersink();
                
        }
    
        // main rounded box indentation
        if(show_indent)
        difference(){
            color("blue")
            
            translate([0, u*1.25 + y_offset, 0])
            make_recharge_rounded_box(
                box_r=box_r,
                box_tol_y=box_tol_y,
                box_tol_xz=box_tol_xz,
                terminal_extension=terminal_extension
            );
    
            // polarity slot
            difference(){
                translate([u*2, 0, 0])
                cube([1, u*1.8, u*5], center=true);
                    
                translate([recharge_mount_terminal_offset, u*1.35-recharge_mount_terminal_extend+terminal_extension+.25+1.1, 0])
                rotate([-90,0,0])
                make_countersink(inner=u*4);
                
            }
        }

        if(show_detail){
            
            // back cutoff
            translate([0,-u*8+.5,0])
            cube([u*30, u*4, u*10], center=true);

            // slot cutout
            translate([0, -u*13.9, 0])
            make_recharge_mount(struct_thicken=tolerance);
            
        }
            
    }//end diff
    
    // terminal standoffs
    if(show_holes)
    difference(){
        union(){
            translate([recharge_mount_terminal_offset, u*1-recharge_mount_terminal_extend+terminal_extension+.25, 0])
            rotate([-90,0,0])
            cylinder(d=screw_head_diameter, h=u*1.5, center=true);
            translate([-recharge_mount_terminal_offset, u*1-recharge_mount_terminal_extend+terminal_extension+.25, 0])
            rotate([-90,0,0])
            cylinder(d=screw_head_diameter, h=u*1.5, center=true);
        }

        translate([recharge_mount_terminal_offset, u*1.35-recharge_mount_terminal_extend+terminal_extension+.25+1.1, 0])
        rotate([-90,0,0])
        make_countersink(inner=u*4);
        translate([-recharge_mount_terminal_offset, u*1.35-recharge_mount_terminal_extend+terminal_extension+.25+1.1, 0])
        rotate([-90,0,0])
        make_countersink(inner=u*4);
        
    }
    
    // reed sensor slot
    if(show_holes)
    translate([5,-3.5+3,0])
    difference(){
    
        cylinder(d=4, h=u*4, center=true);
        cylinder(d=2+tolerance, h=u*5, center=true);
        translate([2,0,u*1.5+1])
        cube([3,1,5], center=true);
        translate([2,0,-u*1.5-1])
        cube([3,1,5], center=true);
        
        translate([0, -3, 0])
        cube([5, 5, u*5], center=true);
    }
     
}

module make_recharge_board_bb(extra_x=0, extra_y=0, extra_z=0){
    cube([70+extra_x, 1.6+extra_y, 20+extra_z], center=true);
}

module make_recharge_led_assembly(){
    translate([0,0,5])
    cylinder(d=6.75, h=10, center=true);
    translate([0,0,-14/2])
    cylinder(d=5, h=14, center=true);
}

module make_recharge_plug_female2(struct_thicken=0, show_holes=1, h_extend=0, box_tol_xz=2.5, show_detail=1, show_indent=1, show_wedge=1){
    
    width = recharge_mount_width;//u*8;
    height = recharge_mount_height;//u*4;
    y_offset = -2;
    terminal_extension = recharge_plug_terminal_extension;
    
    box_r = recharge_plug_box_r;
    //box_tol_xz = 2.5; // little too big
    box_tol_xz = recharge_plug_female_box_tol_xz;
    box_tol_y = recharge_plug_female_box_tol_y;
    
    terminal_inner_length = -2.5;
    
    struct_offset_y = u*.5;
    
    difference(){

        union(){
        
            // outer solid rim
            difference(){
                
                color("purple")
                translate([0, -u*13.5 + y_offset    , 0])
                make_recharge_mount_bb(h_extend=h_extend, back_extend=0);

                // interior rim cutout
                translate([0, -u*13.5 + y_offset, 0])
                cylinder(d=150-u*2+u*.5, h=u*5, center=true, $fn=75);
                    
            }//end diff

            // mount strut bulkhead
            for(i=[-1:2:1])
            translate([
                //u*13*i,
                u*7.5*i,
                -u*4+struct_offset_y/2+.5,
                0])
            cube([u*1.5, u*4+struct_offset_y, u*4], center=true);

        }
        
        if(show_holes){
    
            // mount strut holes
            for(i=[-1:2:1])
            for(j=[-1:2:1])
            translate([u*7.5*i, -u*3, u*1*j])
            rotate([-90, 0, 0])
            make_countersink(d2=screw_head_diameter+tolerance);
            
            translate([recharge_mount_terminal_offset, u*5-recharge_mount_terminal_extend, 0])
            rotate([-90,0,0])
            make_countersink();
            
            translate([-recharge_mount_terminal_offset, u*5-recharge_mount_terminal_extend, 0])
            rotate([-90,0,0])
            make_countersink();
            
        }
    
        // main rounded box indentation
        if(show_indent)
        difference(){
            color("blue")
            
            translate([0, u*1.25 + y_offset, 0])
            make_recharge_rounded_box(
                box_r=box_r,
                box_tol_y=box_tol_y,
                box_tol_xz=box_tol_xz,
                terminal_extension=terminal_extension
            );
    
            // polarity slot
            difference(){
                translate([u*2, 0, 0])
                cube([1, u*1.8, u*5], center=true);
                
                translate([recharge_mount_terminal_offset, u*1.35-recharge_mount_terminal_extend+terminal_extension+.25+1.1, 0])
                rotate([-90,0,0])
                make_countersink(inner=u*4);
                
            }
        }

        if(show_detail){
            
            // back cutoff
            //translate([0,-u*8+.5,0]) cube([u*30, u*4, u*10], center=true);

            // slot cutout
            //translate([0, -u*13.9, 0]) make_recharge_mount(struct_thicken=tolerance);
            
        }
        
        // circuit board slot cutout
        translate([0,-7-2,0]){
            make_recharge_board_bb(extra_z=5, extra_y=0.5);
        }
        
        // led cutout
        translate([0,-(max_footprint/2-5),0])
        rotate([0,0,-45])
        translate([0,max_footprint/2-5,0])
        translate([0,3.5,0])
        rotate([-90,0,0])
        make_recharge_led_assembly();
        
        // center wire cutout
        translate([0,-u*4-1,0])
        cube([85,u*3,5], center=true);
        
        // ext wire grove
        if(show_wedge)
        translate([0, -150/2+7, 0]){
            difference(){
                wedge(h=5, r=150/2-5.25, a=85, $fn=50);
                wedge(h=u*2, r=150/2-15, a=91, $fn=50);
                wedge(h=20, r=200, a=28, $fn=50);
            }
            difference(){
                intersection(){
                    torus(r1=5/2, r2=150/2-5, $fn=50);
                    wedge(h=10, r=150/2, a=85, $fn=50);
                }
                wedge(h=20, r=200, a=28, $fn=50);
            }
        }
        
    }//end diff
    
    // terminal standoffs
    if(show_holes)
    difference(){
        union(){
            translate([recharge_mount_terminal_offset, u*1-recharge_mount_terminal_extend+terminal_extension+.25-terminal_inner_length/2+1.5/2, 0])
            rotate([-90,0,0])
            cylinder(d=screw_head_diameter, h=u*1.5+terminal_inner_length+1.5, center=true);
            
            translate([-recharge_mount_terminal_offset, u*1-recharge_mount_terminal_extend+terminal_extension+.25-terminal_inner_length/2+1.5/2, 0])
            rotate([-90,0,0])
            cylinder(d=screw_head_diameter, h=u*1.5+terminal_inner_length+1.5, center=true);
            
            translate([-recharge_mount_terminal_offset, u*1-1, 0])
            rotate([-90,0,0])
            cylinder(r1=screw_head_diameter*.75, r2=screw_head_diameter*.5, h=4, center=true);
            
            translate([recharge_mount_terminal_offset, u*1-1, 0])
            rotate([-90,0,0])
            cylinder(r1=screw_head_diameter*.75, r2=screw_head_diameter*.5, h=4, center=true);
        }

        translate([recharge_mount_terminal_offset, u*1.35-recharge_mount_terminal_extend+terminal_extension+.25+1.1+1.5, 0])
        rotate([-90,0,0])
        make_countersink(inner=u*4);
        
        translate([-recharge_mount_terminal_offset, u*1.35-recharge_mount_terminal_extend+terminal_extension+.25+1.1+1.5, 0])
        rotate([-90,0,0])
        make_countersink(inner=u*4);

    }//end diff
    
    // reed switch holder
    translate([0,2,-5])
    difference(){
        //main bulk
        translate([0,0,0])
        rotate([0,90,0])
        cylinder(d=10, h=10, center=true);
        //half cut
        translate([0,30/2,0])
        cube([30,30,30], center=true);
        // horizontal slot
        translate([0,0,0])
        cube([30,30,3.6], center=true);
    }
    
     
}

module magnet_95_x_30(h_extend=0, tol=0){
    //9.55x3.09
    local_tol = 0.09;
    cylinder(d=9.46+local_tol+tol, h=3+local_tol+tol+h_extend, center=true);
}

module magnet_95_x_30_slot(h_extend=0, tol=0, vertical_slot_length=30){
    hull(){
        magnet_95_x_30(h_extend=h_extend, tol=tol);
        translate([0,vertical_slot_length,0])
        magnet_95_x_30(h_extend=h_extend, tol=tol);
    }

}

module make_recharge_plug_male2_magnet_slots(tol_mag=1){
 
    terminal_offset = u*2;
    
    magnet_hole_d = 9.55-2;
    
    vertical_slot_length = 0;
    //tol_mag = 1.25;//little too big
    ;//little too big
    
    // main magnet cutouts
    translate([0,-1.5-2.25,u*0]){
        
        translate([terminal_offset, 0, 0]){
            rotate([90,0,0])
            //cylinder(d=9.55+tolerance/2, h=u*2, center=true);
            magnet_95_x_30_slot(h_extend=0, tol=tolerance*tol_mag, vertical_slot_length=vertical_slot_length);
            //cylinder(d=2, h=u*5, center=true);
        }
        
        translate([-terminal_offset, 0, 0]){
            rotate([90,0,0])
            //cylinder(d=9.55+tolerance/2, h=u*2, center=true);
            magnet_95_x_30_slot(h_extend=0, tol=tolerance*tol_mag, vertical_slot_length=vertical_slot_length);
            //cylinder(d=2, h=u*5, center=true);
        }
        
        /*
        translate([0,0,magnet_hole_d/2])
        rotate([0,90,0])
        cylinder(d=2, h=u*10, center=true);
        */
    }
    
    for(i=[-1:2:1]){
        
        // terminal holes
        translate([terminal_offset*i, 2+2.5, 0])
        rotate([90,0,0])
        cylinder(d=magnet_hole_d, h=u*4, center=true);
    
        translate([0,-u*.5,0]){
                
            // wire hole outer
            translate([terminal_offset*i, -u*3+1, -u*0])
            rotate([90,0,0])
            cylinder(d=recharge_plug_wire_d, h=u*3, center=true);
            
            // wire hole inner offset
            translate([terminal_offset*i, -u*1.25, -.75])
            rotate([90,0,0])
            cylinder(d=recharge_plug_wire_d, h=0.52, center=true);
            
            // wire hole inner
            translate([terminal_offset*i, -3.5, u*0])
            rotate([90,0,0])
            cylinder(d=recharge_plug_wire_d, h=u, center=true);
                
        }
        
    }
    
        
}

module make_recharge_plug_male(face_extend=u, half=0){
    
    width = recharge_mount_width;//u*8;
    height = recharge_mount_height;//u*4;
    
    terminal_offset = u*2;
    terminal_extend = 4.5;
    terminal_extension = recharge_plug_terminal_extension;
    
    y_offset = -2;
    
    box_r = recharge_plug_box_r;
    //box_tol_xz = 2.5; // little too big
    box_tol_xz = recharge_plug_male_box_tol_xz;
    box_tol_y = recharge_plug_male_box_tol_y;
    slot_mag = recharge_mount_slot_mag;
    
    // moves the rounded box in (+) or out (-)
    rounded_box_offset_y = 1;
    
    intersection(){
        difference(){
            union(){
                
                difference(){
                    // main bulkhead
                    translate([0,-u/2+face_extend/2,0])
                    cube([width+u, u+face_extend, height], center=true);

                    translate([0,-u*1.1,0]) rotate([0,0,0])
                    make_recharge_plug_female(show_holes=0, show_detail=0, h_extend=1, show_indent=0);
                }
                    
                // extruded rounded cube matching the indentation in the wall            
                translate([0, -u/2 + rounded_box_offset_y, 0])
                make_recharge_rounded_box(
                    box_r=box_r,
                    box_tol_y=box_tol_y,
                    box_tol_xz=box_tol_xz,
                    terminal_extension=terminal_extension
                );

                // wire strain gauge
                color("green")
                translate([0,u*1.5,0])
                cube([u*5, u, u*4], center=true);
                
            }            
            
            /*/ flat back
            color("green")
            translate([0,u*1.5,0])
            cube([width+u+1, u, height+1], center=true);
            */
            
            // polarity slot cutout
            color("red")
            translate([u*2, -5-.4, 0])
            cube([1.5+tolerance, u*1.8, u*5], center=true);

            // screw holes to join halfs
            for(i=[-1:2:1]){
                translate([u*4*i, 0, 0]){
                        
                    color("red")
                    //translate([-1*i, 1.5, u*2-1])
                    translate([0, u*.5, u*2-1])
                    rotate([0,0,0])
                    make_countersink();
                    
                    // arm mount cutouts
                    color("orange")
                    //translate([0, u*3.5-u+tolerance, 0])
                    translate([0, u*3.5-u-tolerance, 0])
                    cube([u+tolerance, u*5, u+tolerance], center=true);
                    
                }
            }
            
            // arm mount holes
            translate([0, u*1.5, u*2-1])
            make_countersink();
            translate([0, u*1.5-u, u*2-1])
            make_countersink();

            // arm mount cutouts
            color("orange")
            //translate([0, u*3.5-u+tolerance, 0])
            translate([0, u*3.5-u-tolerance, 0])
            cube([u+tolerance, u*5, u+tolerance], center=true);
            
            // main magnet cutouts
            color("red")
            //expects indent of 4mm
            //translate([0,0,u*3])
            translate([0, 3, 0])//makes offset expected by terminals
            translate([0, -5.5, 0])//makes flush
            rotate([0,0,180])
            make_recharge_plug_male2_magnet_slots(tol_mag=slot_mag);
            
        }// end diff
        
        translate([0,0,u*5/2*half])
        cube([u*10, u*5, u*5], center=true);
        
    }
}

module make_weight(){
    translate([0, 0, 18/2])
    cylinder(d=130, h=18, center=true);
}

module _make_recharge_dock_front_middle_holes(){
    
    for(i=[-1:2:1]){
        
        translate([(-u*10 + u*.5)*i, u*1.5, -u*.2])
        make_countersink(inner=u*2, outer=u);
        translate([(-u*10 + u*.5)*i, u*4.5, -u*.2])
        make_countersink(inner=u*2, outer=u);
        
        translate([(-u*10 + u*.5)*i, u*1.5, -u*10.3])
        rotate([180,0,0])
        make_countersink(inner=u*2, outer=u);
        translate([(-u*10 + u*.5)*i, u*4.5, -u*10.3])
        rotate([180,0,0])
        make_countersink(inner=u*2, outer=u);
    }
    
}

module make_recharge_dock_spring(){
    cylinder(d=5, h=25, center=true);
}

module _make_recharge_dock_front_top_holes(){
    for(i=[-1:2:1]) for(j=[0:1])
    translate([u*9.5*i, -1.5, u*9-j*u*3])
    rotate([90,0,0])
    make_countersink(inner=2*u, outer=u);
}

module make_recharge_dock_front(){
    
    strut_h = recharge_dock_width/2 - u - u*.5;
    strut_d = u*5;

    difference(){
        union(){
            // main bulkhead
            cube([recharge_dock_width, u, recharge_dock_width], center=true);
            
            // side struts
            for(i=[-1:2:1]){
                translate([(recharge_dock_width/2 - u/2)*i, strut_d/2 + u/2, -strut_h/2 - u - u*.5])
                cube([u, strut_d, strut_h], center=true);
                
            }
        }    
        
        translate([0,0,-u*.5])
        _make_recharge_dock_front_middle_holes();    
        
        // spring cutout
        color("blue")
        translate([-u*0, 0, -u/2 - u*.5])
        scale([3,1,1])
        rotate([90, 0, 0])
        cylinder(d=u*2, h=u*2, center=true);
        
        color("red")
        _make_recharge_dock_front_top_holes();
    
        //color("blue")
        translate([0, -u*1.5, -u*.5 - u*.5])
        make_recharge_dock_arm(solid=0, tol=tolerance);
            
    }
}

module make_recharge_dock_bottom(){
    difference(){
        union(){
            translate([0, -recharge_dock_width/2 + u*2, u*.5/2]){
                difference(){
                    union(){
                        translate([0,0,-u*.5/2])
                        cube([recharge_dock_width, recharge_dock_width, u], center=true);
                        /*
                        color("brown")
                        scale([1,1,.5])
                        rotate([0,90,0])
                        make_cross_plate(
                            w=recharge_dock_width,
                            h=recharge_dock_width,
                            hub_d=0,//u*4,
                            t=u,
                            cross_type=1
                        );
                        */
                    }
                
                    color("red")
                    translate([0, -u*9.5, u*10.3])
                    translate([0,0,-u*.5/2])
                    translate([0,0,-u*0.3])
                    _make_recharge_dock_front_middle_holes();
                }
            }
            
            translate([0,0,u*.9])
            cylinder(d=20, h=u, center=true);
        }
        
        color("gray")
        for(i=[-1:2:1]) for(j=[-1:2:1])
        translate([recharge_dock_width/4*i, -recharge_dock_width/2 + u*2 + recharge_dock_width/4*j, 0])
        cylinder(d=u*4, h=u*2, center=true);
        
        translate([0, -recharge_dock_width/2 + u*2, 0])
        cylinder(d=u*4, h=u*2, center=true);
    }
    
}

module make_recharge_dock_middle(){
    
    difference(){
        cube([recharge_dock_width, recharge_dock_width-u, u], center=true);
        
        color("red")
        translate([0, -u*10, u*.5])
        _make_recharge_dock_front_middle_holes();
        
        translate([0,-u*8,0])
        cube([recharge_dock_width-u*4, recharge_dock_width-u*4, u*2], center=true);
        
        color("red")
        translate([u*6.5,-u,0])
        for(i=[-1:2:1]) for(j=[0:1])
        translate([u*2.75*i, u*5+11*j, 0])
        cylinder(d=3, h=u*5, center=true);
    }
    
    difference(){
        cylinder(d=u*2, h=u*.5, center=true);
        cylinder(d=u*1, h=u*.5+1, center=true);
        translate([3,1.5,0])
        cube([u,u,u], center=true);
    }
    
}

module make_recharge_dock_arm(solid=0, tol=0){
    color("blue")
    difference(){
        union(){
            
            // main oval
            difference(){
                scale([1,2,1]){
                    translate([0,-u,0])
                    difference(){
                        cylinder(d=u*9+tol, h=u*.5+tol, center=true, $fn=50);
                        if(!solid)
                        cylinder(d=u*9-u*3.5, h=u*.5+1, center=true, $fn=50);
                    }
                }
                
                translate([0,-u*5-u*5/2,0])
                cube([u*10, u*15, u*5], center=true);
            }
            
            difference(){
                scale([1.5,2,1]){
                    cylinder(d=u*9, h=u*.5, center=true, $fn=50);
                }
                
                translate([0,-u*1-u*5/2,0])
                cube([u*15, u*15, u*5], center=true);
            }
            
            for(i=[-1:2:1])
            translate([u*4*i, u*.5, u*0]){
                union(){
                    translate([0,0,0])
                    cube([u, u, u*4], center=true);
                    translate([0,-u*.5,0])
                    cube([u, u*2, u], center=true);
                }
            }
        }
        
        for(i=[-1:2:1])
        translate([u*4*i, u*.5, u*0])
        translate([0,-u,0])
        cylinder(d=u*.5, h=u*5, center=true);
        
        // spring cutout
        translate([0,u*6,0])
        rotate([90,0,0])
        cylinder(d=u*4, h=40, center=true);
    
        // wire holes
        color("red")
        for(i=[-1:2:1]) for(j=[0:1])
        translate([u*2.75*i,u*5+u*j,0])
        cylinder(d=3, h=u*5, center=true);
    
    }
    
}

module make_recharge_dock_back(){
}

module make_recharge_dock(){
    
    color("purple")
    translate([0, 0, u*0.5])
    make_weight();
    color("purple")
    translate([0, 0, u*0.5+18])
    make_weight();
    
    color("tan")
    translate([0, 0, 0])
    make_recharge_dock_bottom();
    
    color("red")
    translate([0, -recharge_dock_width/2 - u*8 + u/2, recharge_dock_width/2 + u*.5])
    make_recharge_dock_front();
    
    color("orange")
    translate([0, -recharge_dock_width/2 + u*2.5 -u*0, u/2 + u/2 + u*9])
    translate([0,0,-u*.5])
    make_recharge_dock_middle();
    
    color("green")
    translate([0, -u*15, u*10])
    translate([0,0,-u*.5])
    rotate([90,0,0])
    make_recharge_dock_spring();
    
    color("blue")
    translate([0, -recharge_dock_width*.95, recharge_dock_width/2])
    translate([0,0,-u*.5])
    make_recharge_dock_arm();
    
}

module make_recharge_mount_holes(){
    for(i=[-1:2:1]) for(j=[-1:2:1]) for(k=[0:1])
    rotate([0,0,(50 - 10*k)*j])
    translate([0,u*15-1,u*i])
    rotate([-90,0,0])
    make_countersink();
}

module make_recharge_mount(struct_thicken=0){
    
    width=u*30;
    height=u*4;
    mount_offset_z = recharge_mount_height/2-u/2;//-thickness/2;
    
    translate([0, 0, -u*1.5])
    difference(){
        union(){
            
            //intersection(){
            // bottom horizontal support
            //translate([0,u*11.5,0]) cube([u*10,u,u], center=true);
            
            //}
            
            // vertical supports
            difference(){
                // bulkhead
                color("red")
                translate([0,u*12.25-.5/2+1/2,u*1.5])
                cube([u*9.5 + struct_thicken/2, u*2.5-0.5+1, u*4+struct_thicken], center=true);
                
                // cutout
                color("green")
                translate([0,u*11.5,u*1.5+u])
                cube([u*5,u*5,u*4], center=true);
                
                color("purple")
                translate([0,u*13,u*1.5+u])
                cube([u*9.5-u*2 - struct_thicken/2,u*2,u*7], center=true);
                
                
            }
            
        }
        color("green"){
            translate([u*3,u*11.8,0])
            rotate([-90,0,0])
            make_countersink();
            translate([-u*3,u*11.8,0])
            rotate([-90,0,0])
            make_countersink();
            
            translate([u*3,u*11.8,u*3])
            rotate([-90,0,0])
            make_countersink();
            translate([-u*3,u*11.8,u*3])
            rotate([-90,0,0])
            make_countersink();
        }
        
        /*
        color("red"){
            translate([0,u*13,u*1.5+u*2+struct_thicken/2])
            cube([u*12,u*2,u*2], center=true);
            translate([0,u*13, u*1.5-u*2-struct_thicken/2])
            cube([u*12,u*2,u*2], center=true);
        }*/
        
    }//end diff
    
    difference(){
        cylinder(d=150-u*2, h=u*4, center=true, $fn=100);
        cylinder(d=150-u*4, h=u*5, center=true, $fn=100);
    
        color("blue")
        translate([0, -u*7, 0])
        cube([width, u*30, height*2], center=true);
        
        for(i=[0:1]) for(j=[-1:2:1])
        rotate([0,0,(45-15*i)*j])
        rotate([90,0,0])
        cylinder(d=u*2, h=u*40, center=true);
            
        translate([0,u*13,0])
        cube([u*9.5, u*5, u*5], center=true);

        make_recharge_mount_holes();
        
    }// end diff
    
}

//make_recharge_mount();


translate([0,0,u*2 + 40]){
    translate([0,-u*20,0])
    rotate([0,0,0])
    import("../printable/recharge_plug_male_test_20151128.stl");

    translate([0,-u*22,0])
    import("../printable/recharge_plug_female_20151116.stl");
}

translate([0,0,u*.5])
make_recharge_dock();

//intersection(){
    //translate([0,u*4.5,0])rotate([180,0,0])
    //make_recharge_plug_male(half=0);
    //make_recharge_plug_male(half=-1);
    
/*
    translate([0,-u*1.15,0])//makes flush
    translate([0,-u*2,0])//move out
    rotate([0,0,0])
    make_recharge_plug_female(show_holes=0, show_detail=0);
        //import("../printable/recharge_plug_female_20151116.stl");
//}
*/

