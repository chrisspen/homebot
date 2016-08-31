include <../settings.scad>;
use <countersink.scad>;
use <../electronics/SEN136B5B_Ultrasonic_Sensor.scad>;

module make_sonar_mount_bb(width=u*30, height=u*10, thickness=0, h_extend=0, back_extend=0){
    intersection(){
        color("orange")
        cylinder(d=150, h=height+h_extend, center=true, $fn=100);
        
        color("blue")
        translate([0, u*12-back_extend, 0])
        cube([width, u*8 + back_extend*2, height*2+h_extend], center=true);
    }
}

module make_sonar_shell(){

    difference(){
        color("purple")
        translate([0, -u*13.5 + y_offset, 0])
        make_sonar_mount_bb();
        
        // interior rim cutout
        translate([0, -u*13.5 + y_offset, 0])
        cylinder(d=150-u*2, h=u*10+1, center=true, $fn=75);
    }
}

module make_sonar_mount(){

    sensor_offset_y = 2;
    
    difference(){
        union(){
            make_sonar_shell();
            
            //bulkheads attaching sensors to shell
            for(i=[-1:1:1])
            rotate([0,0,30*i])
            translate([0, u*13.1+sensor_offset_y, 0])
            cube([u*5, u*2, u*10], center=true);
    
            //bulkhead attaching shell to frame
            for(i=[-1:2:1])
            translate([u*3*i, u*12.5, 0])
            cube([u, u*3, u*10], center=true);
        }
        
        // sensor cutouts
  		for(i=[-1:1:1])
		rotate([0,0,30*i])
		translate([0, u*12+sensor_offset_y, 0])
		rotate([-90,0,0])
		//import("electronics/SEN136B5B_Ultrasonic_Sensor.stl");
        make_ultrasonic_sensor(
            cylinder_extend=u*10,
            cylinder_tol=tolerance*2,
            show_mounts=1,
            //mount_height=50,
            smd_tol=2);

        /*/ sensor smd components cutout
        for(i=[-1:1:1])
        rotate([0,0,30*i])
        translate([0, u*12+sensor_offset_y, 0])
        cube([u*5+.1, u*2, u*3], center=true);
        */
    
        // shell mount holes
        color("blue"){
            for(i=[-1:2:1])
            translate([u*3*i, u*14.2-3, -u*2.5])
            rotate([-90,0,0])
            make_countersink();
            
            for(i=[-1:2:1])
            translate([u*3*i, u*14.2-3, u*2.5])
            rotate([-90,0,0])
            make_countersink();
        }
        
    }//end diff
    
}

module make_sonar_cylinder_test(){
    
    difference(){
        color("purple")
        translate([0,0,2])
        cube([u*5, u*10, u*.5], center=true);
        
        make_ultrasonic_sensor(
            cylinder_extend=u*10,
            cylinder_tol=tolerance*2,
            show_mounts=1,
            mount_height=50,
            smd_tol=2);
    }
}

//make_sonar_cylinder_test();
    
make_sonar_mount();

//make_sonar_mount_bb();

/*
        color("red")
  		for(i=[-1:1:1])
		rotate([0,0,30*i])
		translate([0, u*12+2, 0])
		rotate([-90,0,0])
		import("../electronics/SEN136B5B_Ultrasonic_Sensor.stl");
        

*/
