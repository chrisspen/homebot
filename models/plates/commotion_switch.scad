// a small slide lever to be attached to the ComMotion's MCU selection switch, so this switch can be actuated even when the ComMotion is sandwiched between an Arduino and another shield.
module make_commotion_switch(){
    height = 2;
    nub_height = height*2;
    nub_width = 5;
    handle_length = 50;
    grab_depth = 20-5;
    grab_width = 5;
    grab_hole = 2;
    beam_thickness = 2;
    
    difference(){
        
        // grab
        translate([0,-grab_depth/2,-height/2])
        cube([grab_width,grab_depth,height], center=true);
        
        color("red")
        translate([0,-12,0])
        cube([2,1.5,height*5], center=true);
        
    }//end diff
    
    translate([(handle_length-grab_width)/2,0,0]){
        
        // beam
        translate([0,0,-height/2])
        cube([handle_length,beam_thickness,height], center=true);
        
        // nub
        translate([(handle_length-nub_width)/2,0,nub_height/2])
        cube([nub_width,beam_thickness,nub_height], center=true);
        
    }
}

make_commotion_switch();
