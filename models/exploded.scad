
ex = 10;

// Head
translate([0,0, ex*12]){
    
    translate([0,0,0]){
        
        import("printable/bearings_slew_bearing_inner_bottom_20160214.stl");
        
        translate([0,0,ex*5]){
            import("printable/bearings_slew_bearing_outer_20160214.stl");
            
            translate([0,0,ex*3])
            rotate([0,180,0])
            import("printable/bearings_slew_bearing_inner_top_20160214.stl");
        }
    }
}

// Torso
translate([0,0,ex]){

    translate([0,-40+2.5-ex,0])
    rotate([90,0,0])
    import("printable/vertical_battery_slotted_20150915.stl");

    translate([0,40-2.5+ex,0])
    rotate([90,0,180])
    import("printable/vertical_battery_slotted_20150915.stl");

    translate([35+2.5+ex,0,0])
    rotate([90,0,-90])
    import("printable/vertical_back_slotted_20150916.stl");
    
    translate([-ex*2,0,0]){
    
        translate([0,0,2.5-30])
        rotate([0,0,90])
        import("printable/battery_tray2_nub_20160522.stl");
        
        mirror([0,1,0])
        translate([0,0,2.5-30])
        rotate([0,0,90])
        import("printable/battery_tray2_nub_20160522.stl");
        
        translate([0,0,15])
        rotate([0,0,90])
        import("printable/top_rear_panel2_20160526.stl");
        
        translate([-ex*15,0,0]){
            
            translate([0,0,-45])
            rotate([0,0,90])
            import("printable/battery_tray2_cartridge_bottom_20160522.stl");
            
            translate([0,0,ex]){
                translate([0,0,-45+15+2.5])
                rotate([0,180,90])
                import("printable/battery_tray2_cartridge_top_20160522.stl");
            
                translate([0,0,ex*1.5]){
                    //color("blue")
                    translate([11.5,4,-18])
                    rotate([0,0,0])
                    import("printable/screw_2p5_10.stl");
                    
                    //color("blue")
                    translate([-30,4,-18])
                    rotate([0,0,0])
                    import("printable/screw_2p5_10.stl");
                    
                    //color("blue")
                    translate([-71.5,0,-18])
                    rotate([0,0,0])
                    import("printable/screw_2p5_10.stl");
                }
            }
        }
        
        translate([0,0,-45+15+2.5])
        rotate([0,0,90])
        import("printable/battery_tray2_receiver_20160522.stl");
        
    }
    
    //for(i=[-1:2:1]) for(j=[-1:2:1])
    //translate([(40-2.5)*i, (50+1 + ex*3)*j, 0]){
    
    for(j=[0:1]){
        mirror([0,j,0])
        translate([0,-ex*3,0]){
            
            for(i=[0:1])
            mirror([i,0,0])
            translate([(40-2.5), -(50+1), 0])
            rotate([90,0,90])
            import("printable/side_frame_extension_20151129.stl");
            
            translate([0,0,50+ex])
            rotate([0,0,90])
            import("printable/bearings_slew_bearing_outer_mount_20160215.stl");
            
        }
    }
    
    for(r=[0:1])
    mirror([0,r,0])
    translate([0,ex*6,0]){
        translate([0,0,-20])
        rotate([0,0,0])
        import("printable/side_panel_20151204b.stl");

        //color("blue")
        for(i=[-1:2:1])
        translate([25*i,75-10.25+ex*2,40])
        rotate([-90,0,0])
        import("printable/screw_2p5_10.stl");
        
        //color("blue")
        for(i=[-1:2:1])
        translate([30*i,75-12.5+ex*2,-72.5])
        rotate([-90,0,0])
        import("printable/screw_2p5_10.stl");
        
    }
    
    translate([0,-ex*6,0])
    translate([0,0,-20])
    rotate([0,0,180])
    import("printable/side_panel_20151204b.stl");
    
    
    
}

translate([0,0,-45])
rotate([0,0,-90])
import("printable/horizontal_motor_top_slotted_20150915.stl");

translate([ex*2,0,0]){
        
    translate([45+2.5,0,-20])
    rotate([90,0,90])
    import("printable/conduit_beam_20151108.stl");
    
    translate([ex*2,0,0]){
            
        translate([0,0,-20+2.5])
        rotate([0,0,-90])
        import("printable/sonar_mount_2015119.stl");
        
        translate([0,0,25])
        rotate([0,0,-90])
        import("printable/top_front_panel_20151122.stl");
        
        translate([70,0,-50-2.5])
        rotate([0,0,90+45+90])
        import("printable/recharge_plug_female_v2_20160519.stl");
        
            
        translate([0,0,-80])
        rotate([0,0,-90])
        import("printable/edge_sensor_mount_20151110.stl");
        
        translate([ex*3+1,0,-75+2.5])
        rotate([0,180,-90])
        import("printable/edge_sensor_mount_cover_20151111b.stl");
            
    }
    
}

translate([0,0,-ex]){

    translate([0,35+2.5+ex,-65+2.5])
    rotate([90,0,0])
    import("printable/vertical_slotted_20150915.stl");

    translate([0,-(35+2.5)-ex,-65+2.5])
    rotate([90,0,180])
    import("printable/vertical_slotted_20150915.stl");

    translate([0,0,-ex*5]){
        
        translate([0,0,-80])
        rotate([0,0,90])
        import("printable/horizontal_20150726.stl");
        
        translate([0,0,-ex*5]){
            translate([0,0,-90-2.5])
            rotate([0,0,-90])
            import("printable/ballast_tray_20160414.stl");
            
            translate([0,0,-90-2.5])
            rotate([0,0,90])
            import("printable/ballast_tray_20160414.stl");
        
            translate([0,0,-ex*2-87.5]){
                
                translate([-5,40-2.5,0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([-25,40-2.5,0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([-5,-(40-2.5),0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([-25,-(40-2.5),0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([5,40-2.5,0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([25,40-2.5,0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([5,-(40-2.5),0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
                
                
                translate([25,-(40-2.5),0])
                rotate([180,0,0])
                import("printable/screw_2p5_10.stl");
            }
        }
        
    }
    
    translate([-ex,0,0]){
        
        //color("blue")
        translate([-40-1-ex,0,-60-2.5])
        rotate([90,0,-90])
        import("printable/dc_converter_tray_20150917.stl");

        //color("blue")        
        translate([-ex*3,0,-60-2.5])
        rotate([0,0,90])
        import("printable/lower_rear_panel_20151122.stl");
            
    }

}
