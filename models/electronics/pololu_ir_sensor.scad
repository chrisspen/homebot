//Pololu drv8838 single brushed DC motor driver carrier

//https://www.pololu.com/product/1132
//GP2Y0D805Z0F Digital Distance Sensor 5cm
module make_pololu_GP2Y0D805Z0F(height_extend=0, sensor_buffer=0, board_bb=0, x_extend=0, x_offset=0, y_extend=0, hole_offset=0){

    sensor_height = 7.96 + height_extend;

	difference(){
		union(){
    	    //sensor
    	    translate([+1, 0, sensor_height/2+1/2])
    	    cube([13.6+sensor_buffer, 7+sensor_buffer, sensor_height], center=true);
    	    
    	    //board
    	    translate([x_offset, 0, (board_bb==1)?100/2+1/2:0])
    	    cube([21.59+x_extend, 8.89+y_extend, (board_bb==1)?100:1], center=true);
		}
	    
	    //hole
	    translate([+21.59/2-19.05+hole_offset,0,0])
	    cylinder(d=2, h=10, center=true, $fn=100);
	}
}

make_pololu_GP2Y0D805Z0F();
