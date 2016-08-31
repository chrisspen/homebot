include <../settings.scad>;

//
module make_bottom_plate(){
    //cube([105,105,unit_thickness], center=true);
    difference(){
        cylinder(h=unit_thickness, d=base_plate_diameter, center=true);
        
        translate([wheel_offset_x+unit_thickness,0,0])
        cube([wheel_diameter,wheel_diameter+unit_thickness,wheel_diameter], center=true);
        
        translate([-(wheel_offset_x+unit_thickness),0,0])
        cube([wheel_diameter,wheel_diameter+unit_thickness,wheel_diameter], center=true);
        
        hull(){
            translate([0,caster_offset_y,0])
            cylinder(h=caster_holder_diameter, d=caster_holder_diameter, center=true);
            translate([0,caster_offset_y+caster_holder_diameter,0])
            cylinder(h=caster_holder_diameter, d=caster_holder_diameter, center=true);
        }
        
        hull(){
            translate([0,-caster_offset_y,0])
            cylinder(h=caster_holder_diameter, d=caster_holder_diameter, center=true);
            translate([0,-(caster_offset_y+caster_holder_diameter),0])
            cylinder(h=caster_holder_diameter, d=caster_holder_diameter, center=true);
        }
    }
}

// Generates sequential cylinders for punching uniform mounting holes in other shapes.
module make_hole_chain(d, d2=0, h=0, u, w, x=0, y=0, z=0, rx=0, ry=0, rz=0, rx0=0, ry0=0, rz0=0, alternate=0, skip_i=-1){
    /*
    Arguments:
    
        d, d2: start and end diameter of cylinder
        h: height of cylinder, default to u
        u: unit spacing
        w: total width of chain
        x, y, z: overall translation of chain
        rx, ry, rz: overall rotation of chain
        alternate: if 0, only makes holes along one side, if 1 makes holes along both while alternating orientation
    */
    step = alternate ? 2 : 1;
    h = h ? h : u*1.1;
    union(){
        for(i=[0:step:w-1]){
            if(skip_i == -1 || (i % 2 == skip_i)){
                translate([x,y,z])
                rotate([rx, ry, rz])
                translate([-w*u/2+u/2+i*u,0,0])
                //echo("even:", i % 2);
                rotate([90*(i % 2)+rx0,ry0,rz0])
                if(d2){
                    cylinder(d1=d, d2=d2, h=h, center=true);
                }else{
                    cylinder(d=d, h=h, center=true);
                }
            }
        }
    }
}

module make_generic_plate(u, a, b, d=1, countersink=0, cross=0, rotflip=0, border=1){
    // rotflip=1 causes holes to be rotated around strip to match mating shape
    
    A = a*u - u;
    B = b*u - u;
    
    theta = atan(A/B);
    rx0_extra = rotflip * 90;
    
    C = sqrt(A*A + B*B);
    
    difference(){
        union(){
            if(border){
            difference(){
                cube([a*u, b*u, u], center=true);
                cube([a*u-u*2, b*u-u*2, u*2], center=true);
            }
            }
            //color("blue"){
                if(cross){
                    translate([a*u/2-u/2, -b*u/2+u/2, 0])
                    rotate([0,0,theta])
                    translate([0, C/2, 0])
                    cube([u, C, u], center=true);
                    
                    translate([-(a*u/2-u/2), -b*u/2+u/2, 0])
                    rotate([0,0,-theta])
                    translate([0, C/2, 0])
                    cube([u, C, u], center=true);
                }
            //}
        }
                
        color("red"){
            make_hole_chain(d=d, u=u, w=a-1, x=0, y=b*u/2-u/2, z=0, rz=0, rx0=90+rx0_extra);//along x
            make_hole_chain(d=d, u=u, w=b-1, x=-a*u/2+u/2, y=0, z=0, rz=90, rx0=90+rx0_extra);//along y
            make_hole_chain(d=d, u=u, w=a-1, x=0, y=-(b*u/2-u/2), z=0, rz=0, rx0=90+rx0_extra);//along x
            make_hole_chain(d=d, u=u, w=b-1, x=+a*u/2-u/2, y=0, z=0, rz=90, rx0=90+rx0_extra);//along y
        }
    }
}

// Makes a beam along the x-axis.
module make_generic_beam(u, a, b=0, d=1, countersink_d=1, countersink_d2=0, countersink_depth=1, cross=0, rotflip=0, holes=1, cx1=0, cx2=0, cy1=0, cy2=0, end_fills=0, hx=0, hy=0, hz=0, skip_i=-1){
    // u = base unit distance
    // a = number of units of length to use
    // b = 
    // d = 
    
    rx0_extra = rotflip * 90;
    offset = 0;
    //offset = 0.1;//TODO:remove
    length_extension = .5;
    
    // default b to a if not given
    b = b ? b : a;
    
    countersink_h = countersink_depth+length_extension;
    
    //echo("countersink_d:", countersink_d);
    //echo("countersink_d2:", countersink_d2);
    //echo("countersink_depth:", countersink_depth);
    
    union(){
        difference(){
            cube([a*u, u, u], center=true);
            
            if(holes){
                
                color("red"){
                    make_hole_chain(
                        d=d,
                        u=u,
                        w=b-1,
                        
                        x=hx,
                        y=hy,
                        z=hz,
                        
                        rx=0,
                        ry=0,
                        rz=0,
                        
                        rx0=90+rx0_extra,
                        ry0=0,
                        rz0=0,
                        
                        skip_i=skip_i
                    );
                }
                
                color("blue"){
                
                    if(cx1){
                        // laid out along x-axis
                        // hole axis pointing along y-axis
                        // taper towards the +y-axis
                        translate([0, (u/2 + countersink_h/2 - countersink_depth), 0]){
                            make_hole_chain(
                                d=countersink_d+length_extension,
                                d2=countersink_d2,
                                u=u,
                                //w=a-1,
                                w=b-1,
                                h=countersink_h,
                                alternate=1,
                                x=0, y=0, z=0,
                                //x=0, y=0, z=(offset+length_extension/2),
                                rx0=90+rx0_extra, ry=0, rz=0
                            );
                        }
                    }
                    
                    if(cx2){
                        // laid out along x-axis
                        // hole axis pointing along y-axis
                        // taper towards the -y-axis
                        translate([0, -(u/2 + countersink_h/2 - countersink_depth), 0]){
                            make_hole_chain(
                                d=countersink_d+length_extension,
                                d2=countersink_d2,
                                u=u,
                                //w=a-1,
                                w=b-1,
                                h=countersink_h,
                                alternate=1,
                                x=0, y=0, z=0,
                                //x=0, y=0, z=-(offset+length_extension/2),
                                rx0=90+rx0_extra+180, ry=0, rz=0
                            );
                        }
                    }
                    
                    if(cy1){
                        // laid out along y-axis
                        // hole axis pointing along z-axis
                        // taper towards the +z-axis
                        translate([-u/2, 0, (u/2 + countersink_h/2 - countersink_depth)]){
                            make_hole_chain(
                                d=countersink_d+length_extension,
                                d2=countersink_d2,
                                u=u,
                                //w=a-2,
                                w=b-2,
                                h=countersink_h,
                                alternate=1,
                                x=0, y=0, z=0,
                                rx0=0+rx0_extra, ry=180, rz=0
                            );
                        }
                    }
                    
                    if(cy2){
                        // laid out along y-axis
                        // hole axis pointing along z-axis
                        // taper towards the -z-axis
                        translate([u/2, 0, -(u/2 + countersink_h/2 - countersink_depth)]){
                            make_hole_chain(
                                d=countersink_d+length_extension,
                                d2=countersink_d2,
                                u=u,
                                //w=a-2,
                                w=b-2,
                                h=countersink_h,
                                alternate=1,
                                x=0, y=0, z=0,
                                rx0=0+rx0_extra, ry=0, rz=0
                            );
                        }
                    }
                }
                
            }
        
        }//end diff
        
        if(end_fills){
            //color("green")
            translate([-(a*u/2 - end_fills*u/2),0,0])
            cube([end_fills*u, u, u], center=true);
            
            //color("green")
            translate([(a*u/2 - end_fills*u/2),0,0])
            cube([end_fills*u, u, u], center=true);
        }
    
    }
}
