include <settings.scad>;

module make_caster(){
    color("red")
    translate([0,0,0])
    sphere(d=caster_diameter, center=true);
    
    color("blue")
    translate([0,0,25.45/2+13/2])
    rotate([0,180,0])
    scale([1,1,1])
    //difference(){
        import("casters/ball_caster_v2_20150622.stl");
    //    cube([caster_diameter,caster_diameter*2,caster_diameter*2]);
    //}
}

make_caster();
