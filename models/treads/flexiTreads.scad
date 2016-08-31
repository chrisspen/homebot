// Robot tracks w/ flexible fillament
// hristo.staykov@gmail.com

pi = 3.141592653589793238;

//Calculate the variables bellow based on your requirements
//Note that by changing any values, you may also need to compute perfectWheelRadius manually.

trackWidth = 13.5;
trackOffset = 3;        //distance from chassis

wheelRadius = 19;//13.5; //includes the "sprokets" on the wheel

wheelWidth = trackWidth;//8;
wheelOffset = trackOffset + (trackWidth-wheelWidth)/2;
wheelSpacing = 40;//70;
wheelTeeth = 27;//TODO:vary based on wheelSpacing?
wheelSprockets = 17;

// C=2*pi*R + 2*flat
trackLength = 2*wheelRadius*pi + 2*wheelSpacing;

//trackLength = 210;    //Sillicone bracelets usualy have a circumference of 210mm and width of 12mm and can serve as a replacement for the printed tracks. If using a hard-coded value for trackLength, make sure to reflect changes on wheelSpacing.
//wheelSpacing = (trackLength - 2*wheelRadius*3.141592653589793238)/2;

trackRadius = trackLength/2/pi;
trackSprockets = round(wheelSprockets * trackRadius / wheelRadius);

perfectWheelRadius = wheelRadius;//13.5175311111111;  //the closest value to wheelRadius where the wheel sprockets and the track's ones are about the same size.
//TODO: Compute this automatically

//Bolts used for attaching wheels

boltDiameter = 3;	//M3, DIN912, like printrbot's ones
boltHeadHeight = 3;
boltHeadDiameter = 5.5;
boltLength=30;//excl. head
boltUnthreadedSegment=boltLength-18;


module wheel(r, w, offset, motor){
    //Set `motor` to true in order to include a 3mm D-shaft for attaching to pololu micro gearmotors
    inset=2;
    difference(){
	union(){
            rotate_extrude(convexity = 10)
		union(){
			square([r-w/4, w]);
			translate([r-w*1/4, w/2, 0])scale([0.5, 1, 1])circle(w/2, $fn=9);
		}
            difference(){
                translate([0, 0, w])cylinder(r1=5, r2=3, h=offset);
                if(!motor){
                    //cut space for a nut and a washer
                    translate([0, 0, w+offset-3])cylinder(r=5, h=3);
                }
            }//end diff

	}
	union(){
/*
            if (motor){
		translate([0, 0, w+offset-9])motorShaft();
		//the motor wheel will be connected with a module for additional support
            }else{
		
		cylinder(r=1.7, h=offset+w, $fn=32);
		cylinder(r=boltHeadDiameter/2+1, h=(wheelWidth+wheelOffset-boltUnthreadedSegment-3)+1, $fn=32);
		//1 extra mm for a washer
            }
*/

            for(i=[0:wheelSprockets]){
		rotate([0, 0, i*360/wheelSprockets])translate([r, 0, 0])cylinder(r=sprocketSize/2, h=w, $fn=4);
            }
	}	
    }//end diff
}
module wheelCap(inset, width, radius){
	cylinder(h=inset+width, r=5.5/2);
	cylinder(h=width, r=radius);
}
module helperWheel(r, w, wall, offset){
	cylinder(r=r, h=w);
	translate([0, 0, -1])cylinder(r=r+wall, h=1);

}
module motorShaft(){
    //3mm D-shaft for pololu micro gearmotors
    difference(){
	cylinder(h=9,r=1.7, $fn=64);
	translate([-5, 1.25, 0])cube(10);
    }
}


sprocketSize = 4;
module track(r, w=0, fullTrackWidth=0, gripSize=1, gripWidth=0, teeth=0, wr=0, ws=0){
    //Print with flexible fillament
    
    fullTrackWidth = fullTrackWidth ? fullTrackWidth : trackWidth;
    
    C = 2*r*pi;
    //gripSize = 1;//mm
    gripCount = floor(C/2/gripSize);
    //teeth = r;
    teeth = teeth ? teeth : r;
    wr = wr ? wr : wheelRadius;
    ws = ws ? ws : wheelSprockets;
    gripWidth = gripWidth ? gripWidth : fullTrackWidth-2;

    //trackRadius = trackLength/2/pi;
    trackSprockets = round(ws * r / wr);

    echo("fullTrackWidth:", fullTrackWidth);
    difference(){
        cylinder(r=r+1, h=fullTrackWidth, $fn=gripCount);
        cylinder(r=r-w/4, h=fullTrackWidth, $fn=trackSprockets);
        cylinder(r1=r-0.5,r2=r-w/4, h=(fullTrackWidth-w)/2, $fn=trackSprockets);
        translate([0, 0, fullTrackWidth-(fullTrackWidth-w)/2])cylinder(r2=r-0.5,r1=r-w/4, h=(fullTrackWidth-w)/2, $fn=trackSprockets);
        translate([0, 0,  (fullTrackWidth-w)/2]) rotate_extrude(convexity = 10)
		union(){
			square([r-w/4, w]);
			translate([r-w*1/4, w/2, 0])scale([0.5, 1, 1])circle(w/2, $fn=9);
		}
    }//end diff

    translate([0, 0, (fullTrackWidth-w)/2])
        intersection(){
            for(i=[0:trackSprockets]){
		rotate([0, 0, i*360/trackSprockets])translate([r-w*1/4+sprocketSize/2, 0, 0])scale([1, 0.72, 1])cylinder(r=sprocketSize/2, h=w, $fn=4);
            }
            cylinder(r=r, h=w);
        }
    for(i=[0:gripCount]){
        rotate([0, 0, i*360/gripCount])
        translate([r+1, 0, -gripWidth/2+fullTrackWidth/2])
        cylinder(r=gripSize/2, h=gripWidth, $fn=12);
    }

}


//track(trackRadius, wheelWidth);

/*
//mock tread to compare with tread cutout
translate([0,0,50])
color("red")
cylinder(r=trackRadius-.65, h=10, center=true);
*/

translate([0,0,0])wheel(perfectWheelRadius, wheelWidth, wheelOffset, false);

/*
translate([-wheelRadius-2,0,0])
wheel(perfectWheelRadius, wheelWidth, wheelOffset, false);
*/

/*
//mock wheel to compare with spoked wheel
translate([0,0,50])
color("red")
cylinder(r=wheelRadius, h=10, center=true);
*/