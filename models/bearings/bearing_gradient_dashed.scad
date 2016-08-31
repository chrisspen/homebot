include <../settings.scad>;

/*
inner bearing diameter = 95mm
inner bearing circumference = 95*pi = 298.5
slot width = 1mm
strip width = 2mm
total strip height = 4.5mm
inner strip height = 1.75mm
*/
module make_bearing_gradient_dashed(){
    //inner_bearing_diameter = 95+1;
    //inner_bearing_diameter = head_slew_bearing_inner_diameter+1;
    
    inner_bearing_diameter = 300;
    slot_width = 1;
    strip_width = 1;
    total_strip_height = 4.5;
    inner_strip_height = 1.75;
    
    //color("lightgray")
    //square([inner_bearing_diameter, total_strip_height], center=true);

    union(){
        for(i=[0:inner_bearing_diameter/strip_width/2-1]) for(j=[-1:2:1])
            color("black")
            translate([
                -inner_bearing_diameter/2 + strip_width/2 + strip_width*i*2,
                ((total_strip_height/2-inner_strip_height/2)/2 + inner_strip_height/2)*j,
                2
            ])
            //cube([strip_width, total_strip_height, 1], center=true);
            square([strip_width, total_strip_height/2-inner_strip_height/2], center=true);
     
    }

    color("black")
    translate([
        -inner_bearing_diameter/2 + strip_width/2,
        0,
        2
    ])
    square([strip_width, total_strip_height], center=true);
        
}

r = 299*299/84.38/299; // Openscad to Inkscape ratio

for(i=[0:4])
translate([30*i,0,0])
scale([r,r,r])
rotate([0,0,90])
rotate([0,0,25])
make_bearing_gradient_dashed();
