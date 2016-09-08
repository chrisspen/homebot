module exploded_motor(ex=1){
    
    translate([0,0,ex*2]){
        
        translate([-40,-17.5,-36.5+2.5])
        rotate([0,-90,0])
        import("electronics/pololu_drive_motor.stl");
    
        translate([0,0,ex*2]){
            translate([-45,0,-1.25])
            //color("red")
            translate([0,0,-2.5])
            rotate([180,0,0])
            rotate([0,0,90])
            import("printable/motor_unimount_2_a0_20160406.stl");
            
            translate([0,0,ex*2]){
                    
                translate([-15,-33,0])
                import("printable/flexiTreads_idler.stl");
                
                translate([-15,33,0])
                import("printable/flexiTreads_idler.stl");
                
                translate([-40,-18,0])
                import("printable/flexiTreads_driver_20150801.stl");
                
                translate([-40,20,5-.5])
                //color("blue")
                import("printable/flexiTreads_tensioner_18_10_20160529.stl");
            
                translate([0,0,ex*3]){
                        
                    //color("black")
                    translate([0,0,-46])
                    import("printable/mock_tread.stl");
                    
                    translate([0,0,ex*3])
                    translate([-45,0,-1.25])
                    //color("blue")
                    translate([0,0,15])
                    rotate([180,0,0])
                    rotate([0,0,90])
                    import("printable/motor_unimount_2_b0_20160406.stl");
                        
                }
                
            }
        }
        
    }
    
}

exploded_motor(ex=10);
