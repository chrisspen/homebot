include <../settings.scad>;
use <plate.scad>;

module make_caster_mount(){
    u = unit_thickness;
    width = caster_plate_width;
    d = screw_thread_diameter;
    
    // Main beam.
    make_generic_beam(u=u, a=width/u, d=d, end_fills=2.5);
    
    // Front beam.
    translate([0,-(caster_plate_width_minor-u),0])
    rotate([0,0,0])
    make_generic_beam(u=u, a=caster_plate_width_minor/u, d=d, rotflip=1);

    // Left mount point.
    translate([u*(width/u)/2-u-u/2,u, -caster_plate_mount_length/2 + u/2])
    rotate([0,90,0])
    make_generic_beam(
        u=u,
        a=caster_plate_mount_length/u,
        d=d,
        rotflip=1,
        cx2=0,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height
    );
    
    // Right mount point.
    translate([-(u*(width/u)/2-u-u/2),u, -caster_plate_mount_length/2 + u/2])
    rotate([0,90,0])
    make_generic_beam(
        u=u,
        a=caster_plate_mount_length/u,
        d=d,
        rotflip=1,
        cx1=0,
        countersink_d=screw_head_diameter,
        countersink_d2=screw_thread_diameter,
        countersink_depth=screw_head_height
    );
    
    // Mount point curve support.
	color("green")
    difference(){
        union(){
            translate([u*(width/u)/2-u-u/2, u-u, -caster_plate_mount_length/2 + u/2 - u/2])
            rotate([0,90,0])
            make_generic_beam(u=u, a=(caster_plate_mount_length-u)/u, d=d, rotflip=1, holes=0);
            
            translate([-(u*(width/u)/2-u-u/2), u-u, -caster_plate_mount_length/2 + u/2 - u/2])
            rotate([0,90,0])
            make_generic_beam(u=u, a=(caster_plate_mount_length-u)/u, d=d, rotflip=1, holes=0);
        }
        color("blue")
        translate([0,-u/2,-u/2 - (caster_plate_mount_length-u)])
        rotate([0,90,0])
        scale([6,2,1])
        cylinder(h=width, d=u, center=true);
    }
    
    // Angle supports.
    //angle0 = 45;
    adj = width/2 - caster_plate_width_minor/2;
    opp = caster_plate_width_minor - u;
    hyp = sqrt(adj*adj + opp*opp);
    angle0 = atan(opp/adj);
    //color("blue"){
        translate([width/2,-u/2,0])
        rotate([0,0,-angle0])
        translate([0,-u/2,0])
        translate([-u/2,-caster_plate_width_minor/2,0])
        rotate([0,0,90])
        rotate([90,0,0])
        make_generic_beam(
			u=u,
			a=hyp/u,
			d=d,
			rotflip=0,
			end_fills=1,
			hx=u/2);
        
        translate([-width/2,-u/2,0])
        rotate([0,0,angle0])
        translate([0,-u/2,0])
        translate([+u/2,-caster_plate_width_minor/2,0])
        rotate([0,0,90])
        rotate([90,0,0])
        make_generic_beam(
			u=u,
			a=hyp/u,
			d=d,
			rotflip=0,
			end_fills=1,
			hx=u/2);
    //}
}

rotate([0,180,0])
make_caster_mount();

/*
color("red")
translate([20,-20,0])
cylinder(d=2.5, h=100, center=true);

color("red")
translate([10,-20,0])
cylinder(d=2.5, h=100, center=true);

color("red")
translate([25,-15,0])
cylinder(d=2.5, h=100, center=true);
*/