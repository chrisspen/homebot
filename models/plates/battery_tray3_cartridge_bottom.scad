use <battery_tray3.scad>;

difference(){
    make_battery_tray3_cartridge_bottom(show_holes=1);

    translate([0,0,100/2])
    cube([100,100,100],center=true);
}

//make_battery_tray_receiver(negative=1, bb=1);

if(0)
translate([-0,31,-17.5]){
    color("green")
    translate([0,0,10])
    import("../printable/battery_tray3_holder.stl");

    color("blue")
    translate([-30.5,0,4.75])
    rotate([0,180,0])
    import("../printable/battery_tray3_support.stl");
}
